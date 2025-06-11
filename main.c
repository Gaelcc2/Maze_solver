/*
 * Improved Maze Navigation with Enhanced Intersection Handling
 *
 * Key improvements:
 * 1. Better intersection detection using weighted sensor analysis
 * 2. Improved decision making with path history tracking
 * 3. Enhanced dead-end detection and recovery
 * 4. More robust turn completion detection
 * 5. Adaptive speed control for different situations
 */

#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include <stdbool.h>

// ========== ENHANCED CONFIGURATION ==========
#define MAX_SPEED 3000
#define DEFAULT_ROBOT_SPEED 55

// Enhanced timing constants
#define PLACEMENT_WAIT_TIME 5000
#define INITIAL_FORWARD_TIME 750
#define LINE_LOST_WAIT_TIME 200        // Reduced for faster response
#define INTERSECTION_PAUSE_TIME 500    // Reduced for smoother navigation
#define MIN_TURN_TIME 1800
#define MAX_TURN_TIME 2300
#define INTERSECTION_ADVANCE_TIME 300  // Time to advance into intersection

// Enhanced sensor groupings
#define ALL_SENSORS 0xFF
#define CENTER_6_SENSORS 0x7E
#define MIDDLE_4_SENSORS 0x3C
#define OUTER_SENSORS 0x81
#define LEFT_SENSORS 0xF0
#define RIGHT_SENSORS 0x0F

// Speed settings for different maneuvers
#define FULL_SPEED 100
#define INTERSECTION_SPEED 70          // Slower speed at intersections
#define TURN_SLOW_SPEED 55
#define TURN_SHARP_SPEED 40
#define SEARCH_SPEED 60

// ========== ENHANCED TYPE DEFINITIONS ==========

typedef enum {
    LINE_CENTERED,
    LINE_SLIGHT_LEFT,
    LINE_SLIGHT_RIGHT,
    LINE_SHARP_LEFT,
    LINE_SHARP_RIGHT,
    LINE_LOST,
    INTERSECTION_DETECTED,
    DEAD_END_DETECTED          // New state for dead-end detection
} LinePosition;

typedef enum {
    ROBOT_WAITING,
    ROBOT_PLACEMENT_WAIT,
    ROBOT_INITIAL_FORWARD,
    ROBOT_FOLLOWING,
    ROBOT_STOPPED,
    ROBOT_LINE_LOST_WAIT,
    ROBOT_BACKWARD_SEARCH,
    ROBOT_BACKWARD_TO_INTERSECTION,
    ROBOT_INTERSECTION_ADVANCE,    // New state for intersection handling
    ROBOT_POST_TURN_SEARCH        // New state for post-turn line search
} RobotState;

typedef enum {
    INTERSECTION_LEFT,
    INTERSECTION_RIGHT,
    INTERSECTION_STRAIGHT,
    INTERSECTION_T_LEFT,          // T-intersection with only left option
    INTERSECTION_T_RIGHT,         // T-intersection with only right option
    INTERSECTION_DEAD_END,        // Dead end detected
    INTERSECTION_UNKNOWN
} IntersectionDirection;

// ========== PATH TRACKING ==========
#define MAX_PATH_HISTORY 20

typedef enum {
    TURN_LEFT,
    TURN_RIGHT,
    TURN_STRAIGHT
} TurnDirection;

typedef struct {
    TurnDirection turns[MAX_PATH_HISTORY];
    uint8_t turn_count;
    uint8_t current_index;
} PathHistory;

// ========== GLOBAL VARIABLES ==========
uint8_t robot_speed_percent = DEFAULT_ROBOT_SPEED;
bool first_intersection_after_start = true;
uint8_t intersection_count = 0;
PathHistory path_history = {0};
uint8_t consecutive_line_lost_count = 0;
uint32_t last_good_line_time = 0;

void (*TimerA2Task)(void);

// ========== ENHANCED FUNCTION PROTOTYPES ==========

// Path tracking functions
void Add_Turn_To_History(TurnDirection turn);
TurnDirection Get_Opposite_Turn(TurnDirection turn);
bool Should_Try_Alternative_Path(void);

// Enhanced sensor functions
uint8_t Read_IR_Sensors_Multiple(uint8_t sensor_mask, uint8_t samples);
LinePosition Get_Enhanced_Line_Position(void);
bool Is_Enhanced_Intersection(uint8_t reading);
bool Is_Dead_End(uint8_t reading);
IntersectionDirection Analyze_Intersection_Options(uint8_t reading);
uint8_t Count_Active_Sensors(uint8_t reading);
uint8_t Get_Sensor_Weight_Score(uint8_t reading);

// Enhanced navigation functions
void Execute_Smart_Turn(IntersectionDirection direction);
void Execute_Intersection_Advance(void);
bool Search_For_Line_Post_Turn(void);
void Execute_Dead_End_Recovery(void);

// Adaptive speed functions
void Set_Speed_For_Situation(uint8_t base_speed);
uint16_t Calculate_Adaptive_Speed(uint8_t percent_of_max, uint8_t situation_modifier);

// Enhanced debugging
void Debug_Enhanced_Sensor_Pattern(uint8_t reading);
void Debug_Intersection_Analysis(uint8_t reading, IntersectionDirection direction);

// ========== ENHANCED SENSOR READING ==========

/*
 * Read_IR_Sensors_Multiple - Take multiple sensor readings for better accuracy
 * Reduces noise and provides more reliable intersection detection
 */
uint8_t Read_IR_Sensors_Multiple(uint8_t sensor_mask, uint8_t samples) {
    uint8_t readings[5];
    uint8_t i, j;
    
    // Take multiple readings
    for (i = 0; i < samples && i < 5; i++) {
        // Turn on selected IR emitters
        P5->OUT |= (sensor_mask & 0x55);
        P9->OUT |= (sensor_mask & 0xAA);

        // Charge phase
        P7->DIR = sensor_mask;
        P7->OUT = sensor_mask;
        Clock_Delay1us(15);

        // Discharge phase
        P7->DIR = 0x00;
        Clock_Delay1us(800);

        readings[i] = P7->IN & sensor_mask;

        // Turn off emitters
        P5->OUT &= ~0x55;
        P9->OUT &= ~0xAA;
        
        if (i < samples - 1) Clock_Delay1us(100);
    }
    
    // Return the reading with the most active sensors (most reliable)
    uint8_t best_reading = readings[0];
    uint8_t max_count = Count_Active_Sensors(readings[0]);
    
    for (i = 1; i < samples; i++) {
        uint8_t count = Count_Active_Sensors(readings[i]);
        if (count > max_count) {
            max_count = count;
            best_reading = readings[i];
        }
    }
    
    return best_reading;
}

// ========== ENHANCED ANALYSIS FUNCTIONS ==========

uint8_t Count_Active_Sensors(uint8_t reading) {
    uint8_t count = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (reading & (1 << i)) count++;
    }
    return count;
}

uint8_t Get_Sensor_Weight_Score(uint8_t reading) {
    // Give more weight to center sensors
    uint8_t score = 0;
    if (reading & 0x08) score += 4; // Sensor 3
    if (reading & 0x10) score += 4; // Sensor 4
    if (reading & 0x04) score += 3; // Sensor 2
    if (reading & 0x20) score += 3; // Sensor 5
    if (reading & 0x02) score += 2; // Sensor 1
    if (reading & 0x40) score += 2; // Sensor 6
    if (reading & 0x01) score += 1; // Sensor 0
    if (reading & 0x80) score += 1; // Sensor 7
    return score;
}

/*
 * Is_Enhanced_Intersection - Improved intersection detection
 * Uses multiple criteria for more reliable detection
 */
bool Is_Enhanced_Intersection(uint8_t reading) {
    uint8_t count = Count_Active_Sensors(reading);
    uint8_t weight_score = Get_Sensor_Weight_Score(reading);
    
    // Count contiguous groups
    uint8_t groups = 0;
    bool in_group = false;
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        bool current_sensor = (reading & (1 << i)) != 0;
        if (current_sensor && !in_group) {
            groups++;
            in_group = true;
        } else if (!current_sensor && in_group) {
            in_group = false;
        }
    }
    
    // Enhanced intersection criteria
    bool wide_pattern = (count >= 5);
    bool center_heavy = (weight_score >= 10);
    bool spans_center = ((reading & 0x18) != 0); // Sensors 3 or 4 active
    bool outer_sensors = ((reading & 0x81) != 0); // Outer sensors active
    
    return (wide_pattern && spans_center) || 
           (center_heavy && count >= 4) || 
           (outer_sensors && count >= 4 && groups <= 2);
}

/*
 * Is_Dead_End - Detect dead-end situations
 * Looks for narrow line ending patterns
 */
bool Is_Dead_End(uint8_t reading) {
    uint8_t count = Count_Active_Sensors(reading);
    
    // Dead end characteristics: few sensors, no wide pattern
    if (count == 0) return true; // Complete line loss
    if (count <= 2 && !Is_Enhanced_Intersection(reading)) {
        consecutive_line_lost_count++;
        return consecutive_line_lost_count > 3;
    }
    
    consecutive_line_lost_count = 0;
    return false;
}

/*
 * Analyze_Intersection_Options - Advanced intersection analysis
 * Determines available paths and recommends direction
 */
IntersectionDirection Analyze_Intersection_Options(uint8_t reading) {
    if (!Is_Enhanced_Intersection(reading)) {
        if (Is_Dead_End(reading)) return INTERSECTION_DEAD_END;
        return INTERSECTION_UNKNOWN;
    }
    
    // Special handling for first intersection
    if (first_intersection_after_start) {
        printf("FIRST INTERSECTION - FORCING RIGHT TURN\n");
        return INTERSECTION_RIGHT;
    }
    
    // Analyze sensor distribution
    uint8_t left_count = 0, right_count = 0, center_count = 0;
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        if (reading & (1 << i)) {
            if (i <= 2) right_count++;
            else if (i >= 5) left_count++;
            else center_count++;
        }
    }
    
    // Check for T-intersections
    bool has_left = (left_count >= 2);
    bool has_right = (right_count >= 2);
    bool has_straight = (center_count >= 2);
    
    printf("Intersection analysis: L=%d R=%d C=%d (has: L=%d R=%d S=%d)\n", 
           left_count, right_count, center_count, has_left, has_right, has_straight);
    
    // Decision logic with path history consideration
    if (has_left && has_right && has_straight) {
        // Full intersection - use smart path selection
        if (Should_Try_Alternative_Path()) {
            return (left_count > right_count) ? INTERSECTION_LEFT : INTERSECTION_RIGHT;
        } else {
            return INTERSECTION_STRAIGHT;
        }
    } else if (has_left && has_right) {
        // T-intersection without straight option
        return (left_count > right_count) ? INTERSECTION_LEFT : INTERSECTION_RIGHT;
    } else if (has_left && !has_right) {
        return INTERSECTION_T_LEFT;
    } else if (has_right && !has_left) {
        return INTERSECTION_T_RIGHT;
    } else if (has_straight) {
        return INTERSECTION_STRAIGHT;
    } else {
        return INTERSECTION_UNKNOWN;
    }
}

// ========== PATH TRACKING FUNCTIONS ==========

void Add_Turn_To_History(TurnDirection turn) {
    if (path_history.turn_count < MAX_PATH_HISTORY) {
        path_history.turns[path_history.turn_count] = turn;
        path_history.turn_count++;
    } else {
        // Shift array left and add new turn
        uint8_t i;
        for (i = 0; i < MAX_PATH_HISTORY - 1; i++) {
            path_history.turns[i] = path_history.turns[i + 1];
        }
        path_history.turns[MAX_PATH_HISTORY - 1] = turn;
    }
}

TurnDirection Get_Opposite_Turn(TurnDirection turn) {
    switch (turn) {
        case TURN_LEFT: return TURN_RIGHT;
        case TURN_RIGHT: return TURN_LEFT;
        default: return TURN_STRAIGHT;
    }
}

bool Should_Try_Alternative_Path(void) {
    // Simple algorithm: prefer right turns initially, then alternate
    return (intersection_count % 3) != 0;
}

// ========== ENHANCED LINE POSITION DETECTION ==========

LinePosition Get_Enhanced_Line_Position(void) {
    uint8_t reading = Read_IR_Sensors_Multiple(CENTER_6_SENSORS, 2);
    uint8_t all_reading = Read_IR_Sensors_Multiple(ALL_SENSORS, 2);
    
    // Check for special situations first
    if (Is_Enhanced_Intersection(all_reading)) return INTERSECTION_DETECTED;
    if (Is_Dead_End(all_reading)) return DEAD_END_DETECTED;
    
    // Extract sensor readings
    bool sensor1 = (reading & 0x02) != 0;
    bool sensor2 = (reading & 0x04) != 0;
    bool sensor3 = (reading & 0x08) != 0;
    bool sensor4 = (reading & 0x10) != 0;
    bool sensor5 = (reading & 0x20) != 0;
    bool sensor6 = (reading & 0x40) != 0;
    
    // Enhanced position detection with weighted scoring
    uint8_t weight_score = Get_Sensor_Weight_Score(reading);
    
    if (sensor3 && sensor4) {
        last_good_line_time = 0; // Reset line lost counter
        return LINE_CENTERED;
    } else if (sensor3 && !sensor4) {
        last_good_line_time = 0;
        return LINE_SLIGHT_RIGHT;
    } else if (sensor4 && !sensor3) {
        last_good_line_time = 0;
        return LINE_SLIGHT_LEFT;
    } else if (sensor2 || (sensor1 && weight_score >= 3)) {
        last_good_line_time = 0;
        return LINE_SHARP_RIGHT;
    } else if (sensor5 || (sensor6 && weight_score >= 3)) {
        last_good_line_time = 0;
        return LINE_SHARP_LEFT;
    } else {
        last_good_line_time++;
        return LINE_LOST;
    }
}

// ========== ENHANCED NAVIGATION FUNCTIONS ==========

void Execute_Smart_Turn(IntersectionDirection direction) {
    TurnDirection turn_made;
    
    switch (direction) {
        case INTERSECTION_LEFT:
        case INTERSECTION_T_LEFT:
            printf("EXECUTING SMART LEFT TURN\n");
            Execute_Left_Turn();
            turn_made = TURN_LEFT;
            break;
            
        case INTERSECTION_RIGHT:
        case INTERSECTION_T_RIGHT:
            printf("EXECUTING SMART RIGHT TURN\n");
            Execute_Right_Turn();
            turn_made = TURN_RIGHT;
            break;
            
        case INTERSECTION_STRAIGHT:
            printf("CONTINUING STRAIGHT\n");
            Move_Forward();
            Clock_Delay1ms(400);
            turn_made = TURN_STRAIGHT;
            break;
            
        case INTERSECTION_DEAD_END:
            printf("DEAD END DETECTED - INITIATING RECOVERY\n");
            Execute_Dead_End_Recovery();
            return; // Don't add to history for dead end
            
        default:
            printf("UNKNOWN INTERSECTION - DEFAULTING TO RIGHT\n");
            Execute_Right_Turn();
            turn_made = TURN_RIGHT;
            break;
    }
    
    Add_Turn_To_History(turn_made);
}

void Execute_Dead_End_Recovery(void) {
    printf("EXECUTING DEAD END RECOVERY SEQUENCE\n");
    
    // Stop and analyze
    Stop();
    Clock_Delay1ms(500);
    
    // Try to back up and find the line
    printf("BACKING UP TO FIND INTERSECTION\n");
    Move_Backward();
    Clock_Delay1ms(800);
    
    // Look for intersection while backing up
    uint32_t backup_timer = 0;
    while (backup_timer < 40) { // 2 seconds max
        uint8_t reading = Read_IR_Sensors_Multiple(ALL_SENSORS, 2);
        
        if (Is_Enhanced_Intersection(reading)) {
            printf("FOUND INTERSECTION DURING BACKUP\n");
            Stop();
            Clock_Delay1ms(300);
            
            // Analyze and turn
            IntersectionDirection direction = Analyze_Intersection_Options(reading);
            if (direction == INTERSECTION_LEFT || direction == INTERSECTION_T_LEFT) {
                Execute_Left_Turn_From_Backward();
            } else {
                Execute_Right_Turn_From_Backward();
            }
            return;
        }
        
        Clock_Delay1ms(50);
        backup_timer++;
    }
    
    // If no intersection found, try a 180-degree turn
    printf("NO INTERSECTION FOUND - EXECUTING 180 TURN\n");
    Stop();
    Clock_Delay1ms(200);
    
    Left_Backward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));
    Clock_Delay1ms(3800); // Longer turn for 180 degrees
    
    Stop();
    Clock_Delay1ms(200);
}

bool Search_For_Line_Post_Turn(void) {
    printf("SEARCHING FOR LINE AFTER TURN\n");
    
    uint8_t search_attempts = 0;
    while (search_attempts < 10) {
        LinePosition pos = Get_Enhanced_Line_Position();
        
        if (pos != LINE_LOST && pos != DEAD_END_DETECTED) {
            printf("LINE FOUND AFTER TURN SEARCH\n");
            return true;
        }
        
        // Small forward movement to search
        Move_Forward();
        Clock_Delay1ms(100);
        search_attempts++;
    }
    
    printf("LINE NOT FOUND AFTER TURN - MAY NEED RECOVERY\n");
    return false;
}

// ========== ENHANCED DEBUG FUNCTIONS ==========

void Debug_Enhanced_Sensor_Pattern(uint8_t reading) {
    uint8_t i;
    printf("Sensors: ");
    for (i = 7; i >= 0; i--) {
        printf("%c", (reading & (1 << i)) ? 'X' : '.');
    }
    
    uint8_t count = Count_Active_Sensors(reading);
    uint8_t weight = Get_Sensor_Weight_Score(reading);
    
    printf(" Count:%d Weight:%d", count, weight);
    
    if (Is_Enhanced_Intersection(reading)) {
        printf(" -> INTERSECTION");
    } else if (Is_Dead_End(reading)) {
        printf(" -> DEAD_END");
    } else if (count >= 1) {
        printf(" -> LINE");
    } else {
        printf(" -> NO_LINE");
    }
    printf("\n");
}

void Debug_Intersection_Analysis(uint8_t reading, IntersectionDirection direction) {
    printf("=== INTERSECTION ANALYSIS ===\n");
    Debug_Enhanced_Sensor_Pattern(reading);
    
    printf("Direction: ");
    switch (direction) {
        case INTERSECTION_LEFT: printf("LEFT\n"); break;
        case INTERSECTION_RIGHT: printf("RIGHT\n"); break;
        case INTERSECTION_STRAIGHT: printf("STRAIGHT\n"); break;
        case INTERSECTION_T_LEFT: printf("T-LEFT\n"); break;
        case INTERSECTION_T_RIGHT: printf("T-RIGHT\n"); break;
        case INTERSECTION_DEAD_END: printf("DEAD_END\n"); break;
        default: printf("UNKNOWN\n"); break;
    }
    
    printf("Intersection count: %d\n", intersection_count);
    printf("Path history: ");
    uint8_t i;
    for (i = 0; i < path_history.turn_count && i < 5; i++) {
        switch (path_history.turns[i]) {
            case TURN_LEFT: printf("L "); break;
            case TURN_RIGHT: printf("R "); break;
            case TURN_STRAIGHT: printf("S "); break;
        }
    }
    printf("\n=============================\n");
}

// ========== MAIN PROGRAM WITH ENHANCEMENTS ==========

/*
 * Enhanced main function with improved state handling
 * Key improvements:
 * - Better intersection detection and handling
 * - Path tracking and smart decision making
 * - Enhanced dead-end recovery
 * - More robust line following
 */

// Note: Include all the original hardware initialization functions here
// (Motor_Init, IR_Init, PWM functions, LED functions, etc.)
// They remain the same as in your original code

// Enhanced main loop - replaces your original main function
int main(void) {
    // Initialize hardware systems
    Clock_Init48MHz();
    Motor_Init();
    IR_Init();
    Set_Robot_Speed(DEFAULT_ROBOT_SPEED);

    // Initialize enhanced state machine variables
    RobotState current_state = ROBOT_WAITING;
    uint8_t start_counter = 0;
    static uint32_t blink_counter = 0;
    bool arrived_from_backward = false;
    uint32_t placement_timer = 0;
    uint32_t forward_timer = 0;
    uint32_t line_lost_timer = 0;
    uint32_t post_turn_timer = 0;

    // Initialize enhanced tracking
    consecutive_line_lost_count = 0;
    last_good_line_time = 0;
    
    LED_Red_On();

    while (1) {
        switch (current_state) {

            case ROBOT_WAITING:
                if (Check_Start_Line()) {
                    start_counter++;
                    if (start_counter >= 3) {
                        printf("START DETECTED - BEGINNING ENHANCED NAVIGATION\n");
                        current_state = ROBOT_PLACEMENT_WAIT;
                        LED_Magenta_On();
                        placement_timer = 0;
                        start_counter = 0;
                        
                        // Reset enhanced tracking
                        arrived_from_backward = false;
                        first_intersection_after_start = true;
                        intersection_count = 0;
                        path_history.turn_count = 0;
                        consecutive_line_lost_count = 0;
                    }
                } else {
                    start_counter = 0;
                    blink_counter++;
                    if (blink_counter % 20 == 0) {
                        LED_Off();
                        Clock_Delay1ms(100);
                        LED_Red_On();
                    }
                }
                break;

            case ROBOT_PLACEMENT_WAIT:
                LED_Magenta_On();
                placement_timer++;

                if (placement_timer % 20 == 0) {
                    uint8_t seconds_remaining = 5 - (placement_timer / 20);
                    if (seconds_remaining > 0) {
                        printf("ENHANCED PLACEMENT WAIT: %d seconds\n", seconds_remaining);
                    }
                }

                if (placement_timer >= 100) {
                    printf("STARTING ENHANCED INITIAL FORWARD\n");
                    current_state = ROBOT_INITIAL_FORWARD;
                    LED_White_On();
                    forward_timer = 0;
                }
                break;

            case ROBOT_INITIAL_FORWARD:
                LED_White_On();
                Move_Forward();
                forward_timer++;

                if (forward_timer >= 15) {
                    printf("ENHANCED LINE FOLLOWING ACTIVATED\n");
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    forward_timer = 0;
                }
                break;

            case ROBOT_FOLLOWING:
                LED_Blue_On();
                LinePosition line_pos = Get_Enhanced_Line_Position();

                // Enhanced debug output
                static uint8_t debug_counter = 0;
                debug_counter++;
                if (debug_counter % 10 == 0) {
                    uint8_t all_sensors = Read_IR_Sensors_Multiple(ALL_SENSORS, 2);
                    Debug_Enhanced_Sensor_Pattern(all_sensors);
                }

                switch (line_pos) {
                    case INTERSECTION_DETECTED:
                        printf("ENHANCED INTERSECTION DETECTED\n");
                        Stop();
                        current_state = ROBOT_INTERSECTION_ADVANCE;
                        LED_Green_On();
                        break;
                        
                    case DEAD_END_DETECTED:
                        printf("DEAD END DETECTED - INITIATING RECOVERY\n");
                        Stop();
                        Execute_Dead_End_Recovery();
                        current_state = ROBOT_FOLLOWING;
                        LED_Blue_On();
                        break;
                        
                    case LINE_CENTERED: 
                        Move_Forward(); 
                        break;
                        
                    case LINE_SLIGHT_LEFT: 
                        Turn_Slight_Left(); 
                        break;
                        
                    case LINE_SLIGHT_RIGHT: 
                        Turn_Slight_Right(); 
                        break;
                        
                    case LINE_SHARP_LEFT: 
                        Turn_Sharp_Left(); 
                        break;
                        
                    case LINE_SHARP_RIGHT: 
                        Turn_Sharp_Right(); 
                        break;
                        
                    case LINE_LOST:
                        if (last_good_line_time > 6) { // Lost for more than 300ms
                            printf("ENHANCED LINE LOST - STARTING RECOVERY\n");
                            Stop();
                            current_state = ROBOT_LINE_LOST_WAIT;
                            LED_Orange_On();
                            line_lost_timer = 0;
                        } else {
                            // Brief forward movement to search
                            Move_Forward();
                        }
                        break;
                        
                    default: 
                        Move_Forward(); 
                        break;
                }
                break;

            case ROBOT_INTERSECTION_ADVANCE:
                // New state: advance slightly into intersection for better analysis
                LED_Green_On();
                Move_Forward();
                Clock_Delay1ms(INTERSECTION_ADVANCE_TIME);
                Stop();
                
                current_state = ROBOT_STOPPED;
                break;

            case ROBOT_STOPPED:
                LED_Green_On();
                Clock_Delay1ms(INTERSECTION_PAUSE_TIME);

                if (arrived_from_backward) {
                    printf("ENHANCED BACKWARD ARRIVAL HANDLING\n");
                    
                    uint8_t reading = Read_IR_Sensors_Multiple(ALL_SENSORS, 3);
                    Debug_Enhanced_Sensor_Pattern(reading);
                    
                    IntersectionDirection direction = Analyze_Intersection_Options(reading);
                    Debug_Intersection_Analysis(reading, direction);
                    
                    Execute_Smart_Turn(direction);
                    
                    arrived_from_backward = false;
                    current_state = ROBOT_POST_TURN_SEARCH;
                    LED_Cyan_On();
                    post_turn_timer = 0;

                } else {
                    // Enhanced forward intersection handling
                    uint8_t reading = Read_IR_Sensors_Multiple(ALL_SENSORS, 3);
                    printf("ENHANCED INTERSECTION ANALYSIS: ");
                    Debug_Enhanced_Sensor_Pattern(reading);

                    if (Is_Enhanced_Intersection(reading)) {
                        intersection_count++;
                        printf("CONFIRMED INTERSECTION #%d\n", intersection_count);

                        IntersectionDirection direction = Analyze_Intersection_Options(reading);
                        Debug_Intersection_Analysis(reading, direction);
                        
                        Execute_Smart_Turn(direction);

                        if (first_intersection_after_start) {
                            first_intersection_after_start = false;
                            printf("FIRST INTERSECTION COMPLETED\n");
                        }

                        current_state = ROBOT_POST_TURN_SEARCH;
                        LED_Cyan_On();
                        post_turn_timer = 0;
                    } else {
                        printf("FALSE INTERSECTION - RESUMING LINE FOLLOWING\n");
                        current_state = ROBOT_FOLLOWING;
                        LED_Blue_On();
                    }
                }
                break;

            case ROBOT_POST_TURN_SEARCH:
                // New state: brief search for line after turn
                LED_Cyan_On();
                post_turn_timer++;
                
                if (Search_For_Line_Post_Turn() || post_turn_timer >= 6) {
                    printf("POST-TURN SEARCH COMPLETE - RESUMING FOLLOWING\n");
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    post_turn_timer = 0;
                }
                break;

            case ROBOT_LINE_LOST_WAIT:
                LED_Orange_On();
                Stop();
                line_lost_timer++;

                LinePosition wait_line_pos = Get_Enhanced_Line_Position();
                if (wait_line_pos != LINE_LOST && wait_line_pos != DEAD_END_DETECTED) {
                    printf("LINE RECOVERED DURING WAIT\n");
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    line_lost_timer = 0;
