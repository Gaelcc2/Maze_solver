/*
 * Basic Maze Navigation
 *
 * OVERVIEW:
 * This code implements a line-following robot that uses 8 IR sensors to detect
 * and follow a black line on a white surface. The robot can:
 * - Follow lines with proportional steering
 * - Detect and navigate intersections (left/right turns)
 * - Handle dead-ends by backing up and finding alternate routes
 * - Provide visual feedback through RGB LED status indicators
 */

#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include <stdbool.h>

// ========== CONFIGURATION CONSTANTS ==========
#define MAX_SPEED 3000
#define DEFAULT_ROBOT_SPEED 55  // 50% of max speed

// Timing constants for robot behaviors (in milliseconds)
#define PLACEMENT_WAIT_TIME 5000     // User placement adjustment time
#define INITIAL_FORWARD_TIME 750     // Initial forward movement after start
#define LINE_LOST_WAIT_TIME 300      // Stabilization wait after line loss
#define INTERSECTION_PAUSE_TIME 1000 // Pause at intersection for analysis
#define MIN_TURN_TIME 1900          // Minimum turn duration
#define MAX_TURN_TIME 2500          // Maximum turn duration (safety)

// Sensor masks for different sensor groups (8 sensors: 0-7, left to right)
#define ALL_SENSORS 0xFF        // All 8 sensors
#define CENTER_6_SENSORS 0x7E   // Sensors 1-6 (exclude outer sensors)
#define MIDDLE_4_SENSORS 0x3C   // Sensors 2-5 (start detection)

// Motor speed percentages for different maneuvers
#define FULL_SPEED 100
#define TURN_SLOW_SPEED 55
#define TURN_SHARP_SPEED 40

// ========== TYPE DEFINITIONS ==========

/*
 * LINE POSITION ENUM
 * Describes where the line is relative to the robot's sensors
 */
typedef enum {
    LINE_CENTERED,        // Line is centered under robot
    LINE_SLIGHT_LEFT,     // Line slightly to the left - gentle correction needed
    LINE_SLIGHT_RIGHT,    // Line slightly to the right - gentle correction needed
    LINE_SHARP_LEFT,      // Line sharply to the left - strong correction needed
    LINE_SHARP_RIGHT,     // Line sharply to the right - strong correction needed
    LINE_LOST,            // No line detected - robot needs to search
    INTERSECTION_DETECTED // Wide line pattern indicates intersection
} LinePosition;

/*
 * ROBOT STATE MACHINE
 * Controls overall robot behavior with visual LED feedback
 */
typedef enum {
    ROBOT_WAITING,              // Red LED - waiting for start signal
    ROBOT_PLACEMENT_WAIT,       // Magenta LED - 5 second countdown
    ROBOT_INITIAL_FORWARD,      // White LED - initial forward movement
    ROBOT_FOLLOWING,            // Blue LED - actively following line
    ROBOT_STOPPED,              // Green LED - stopped at intersection
    ROBOT_LINE_LOST_WAIT,       // Orange LED - line loss stabilization
    ROBOT_BACKWARD_SEARCH,      // Yellow LED - searching backward for line
    ROBOT_BACKWARD_TO_INTERSECTION  // Cyan LED - backing to intersection
} RobotState;

/*
 * INTERSECTION DIRECTION
 * Determines which way to turn at an intersection
 */
typedef enum {
    INTERSECTION_LEFT,     // Turn left at intersection
    INTERSECTION_RIGHT,    // Turn right at intersection
    INTERSECTION_STRAIGHT, // Go straight through intersection
    INTERSECTION_UNKNOWN   // Cannot determine direction
} IntersectionDirection;

// ========== GLOBAL VARIABLES ==========
uint8_t robot_speed_percent = DEFAULT_ROBOT_SPEED;
bool first_intersection_after_start = true;  // Special handling for first intersection
uint8_t intersection_count = 0;

void (*TimerA2Task)(void);

// ========== FUNCTION PROTOTYPES ==========
// Hardware initialization
void Motor_Init();
void IR_Init();
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4);
void TimerA2_Init(void(*task)(void), uint16_t period);

// Sensor functions
uint8_t Read_IR_Sensors(uint8_t sensor_mask);
LinePosition Get_Line_Position();
bool Is_Intersection(uint8_t reading);
bool Is_Turn_Complete();
bool Check_Start_Line();

// Motor control
void Move(uint16_t leftDuty, uint16_t rightDuty);
void Stop();
void Move_Forward();
void Move_Backward();
void Turn_Slight_Left();
void Turn_Slight_Right();
void Turn_Sharp_Left();
void Turn_Sharp_Right();
void Turn_Slight_Left_Backward();
void Turn_Slight_Right_Backward();
void Turn_Sharp_Left_Backward();
void Turn_Sharp_Right_Backward();

// Navigation functions
void Execute_Left_Turn();
void Execute_Right_Turn();
void Execute_Left_Turn_From_Backward();
void Execute_Right_Turn_From_Backward();
IntersectionDirection Get_Intersection_Direction(uint8_t reading);
IntersectionDirection Get_Intersection_Direction_From_Backward(uint8_t reading);

// Utility functions
void Set_Robot_Speed(uint8_t speed_percent);
uint16_t Calculate_Speed(uint8_t percent_of_max);
void Debug_Sensor_Pattern(uint8_t reading);

// LED functions
void LED_Red_On();
void LED_Green_On();
void LED_Blue_On();
void LED_Yellow_On();
void LED_Cyan_On();
void LED_Magenta_On();
void LED_White_On();
void LED_Orange_On();
void LED_Off();

// Low-level motor control
void Left_Forward();
void Left_Backward();
void Right_Forward();
void Right_Backward();
void PWM_Duty3(uint16_t duty3);
void PWM_Duty4(uint16_t duty4);

// ========== TIMER INTERRUPT SETUP ==========
void TimerA2_Init(void(*task)(void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00) | 0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}

void task() {
    printf("interrupt occurs!\n");
}

// ========== PWM MOTOR CONTROL ==========
/*
 * PWM_Init34 - Initialize PWM for motor speed control
 * Uses Timer A0 to generate PWM signals on P2.6 and P2.7
 * period: PWM period (15000 for 15kHz)
 * duty3/duty4: Initial duty cycles for left/right motors
 */
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4) {
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;

    TIMER_A0->CCTL[0] = 0x800;
    TIMER_A0->CCR[0] = period;
    TIMER_A0->EX0 = 0x0000;
    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;
    TIMER_A0->CTL = 0x02F0;
}

// ========== HARDWARE INITIALIZATION ==========
/*
 * Motor_Init - Initialize motor control pins
 * Sets up sleep pins (P3.6, P3.7), direction pins (P5.4, P5.5), and PWM pins
 */
void Motor_Init(){
    // Motor sleep pins (P3.6, P3.7)
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR  |= 0xC0;
    P3->OUT  &= ~0xC0;

    // Motor direction pins (P5.4, P5.5)
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR  |= 0x30;
    P5->OUT  &= ~0x30;

    // PWM pins (P2.6, P2.7)
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR  |= 0xC0;
    P2->OUT  &= ~0xC0;

    PWM_Init34(15000, 0, 0);  // 15kHz PWM, 0% duty cycle
}

/*
 * IR_Init - Initialize IR sensor system
 * Sets up IR emitters on P5 and P9, sensors on P7, and RGB LED on P2
 */
void IR_Init() {
    // IR Emitters on P5 (even) and P9 (odd)
    P5->SEL0 &= ~0x55;
    P5->SEL1 &= ~0x55;
    P5->DIR  |= 0x55;
    P5->OUT  &= ~0x55;

    P9->SEL0 &= ~0xAA;
    P9->SEL1 &= ~0xAA;
    P9->DIR  |= 0xAA;
    P9->OUT  &= ~0xAA;

    // IR Sensors (P7) - inputs
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR  &= ~0xFF;

    // RGB LED indicators (P2.0, P2.1, P2.2)
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

// ========== SENSOR READING ==========
/*
 * Read_IR_Sensors - Core sensor reading function
 * Uses capacitive charging technique to measure reflectance
 * sensor_mask: which sensors to read (bit pattern)
 * Returns: bit pattern where 1 = line detected, 0 = no line
 */
uint8_t Read_IR_Sensors(uint8_t sensor_mask) {
    // Turn on selected IR emitters
    P5->OUT |= (sensor_mask & 0x55);
    P9->OUT |= (sensor_mask & 0xAA);

    // Charge phase - charge sensor capacitors
    P7->DIR = sensor_mask;
    P7->OUT = sensor_mask;
    Clock_Delay1us(15);

    // Discharge phase - measure discharge time (reflectance)
    P7->DIR = 0x00;
    Clock_Delay1us(800);

    uint8_t sensor_readings = P7->IN & sensor_mask;

    // Turn off emitters to save power
    P5->OUT &= ~0x55;
    P9->OUT &= ~0xAA;

    return sensor_readings;
}

// ========== SPEED CONTROL ==========
void Set_Robot_Speed(uint8_t speed_percent) {
    if (speed_percent >= 1 && speed_percent <= 100) {
        robot_speed_percent = speed_percent;
    }
}

uint16_t Calculate_Speed(uint8_t percent_of_max) {
    uint32_t speed = (MAX_SPEED * robot_speed_percent * percent_of_max) / 10000;
    return (uint16_t)speed;
}

// ========== LED STATUS INDICATORS ==========
// LED colors indicate robot state for debugging and monitoring
void LED_Red_On() { P2->OUT = (P2->OUT & ~0x07) | 0x01; }
void LED_Green_On() { P2->OUT = (P2->OUT & ~0x07) | 0x02; }
void LED_Blue_On() { P2->OUT = (P2->OUT & ~0x07) | 0x04; }
void LED_Yellow_On() { P2->OUT = (P2->OUT & ~0x07) | 0x03; }
void LED_Cyan_On() { P2->OUT = (P2->OUT & ~0x07) | 0x06; }
void LED_Magenta_On() { P2->OUT = (P2->OUT & ~0x07) | 0x05; }
void LED_White_On() { P2->OUT = (P2->OUT & ~0x07) | 0x07; }
void LED_Orange_On() { P2->OUT = (P2->OUT & ~0x07) | 0x03; }
void LED_Off() { P2->OUT &= ~0x07; }

// ========== DEBUG AND ANALYSIS ==========
/*
 * Debug_Sensor_Pattern - Print sensor readings in human-readable format
 * Shows which sensors detect the line and analyzes the pattern
 */
void Debug_Sensor_Pattern(uint8_t reading) {
    int i;
    printf("Sensors: ");
    for (i = 7; i >= 0; i--) {
        printf("%c", (reading & (1 << i)) ? 'X' : '.');
    }

    // Count active sensors and sensor groups
    uint8_t count = 0, groups = 0;
    bool in_group = false;

    for (i = 0; i < 8; i++) {
        if (reading & (1 << i)) {
            count++;
            if (!in_group) {
                groups++;
                in_group = true;
            }
        } else {
            in_group = false;
        }
    }

    printf(" Count:%d Groups:%d", count, groups);

    // Classify the pattern
    if (count >= 5 && groups <= 2) {
        printf(" -> INTERSECTION");
    } else if (count >= 1 && count <= 4) {
        printf(" -> LINE");
    } else {
        printf(" -> NO_LINE");
    }
    printf("\n");
}

// ========== INTERSECTION DETECTION ==========
/*
 * Is_Intersection - Detect intersection by analyzing sensor pattern
 * Intersections show as wide line patterns (many sensors active)
 * Returns true if pattern indicates an intersection
 */
bool Is_Intersection(uint8_t reading) {
    int i;
    uint8_t count = 0, groups = 0;
    bool in_group = false;

    // Count total active sensors
    for (i = 0; i < 8; i++) {
        if (reading & (1 << i)) count++;
    }

    // Count contiguous groups of sensors
    for (i = 0; i < 8; i++) {
        bool current_sensor = (reading & (1 << i)) != 0;
        if (current_sensor && !in_group) {
            groups++;
            in_group = true;
        } else if (!current_sensor && in_group) {
            in_group = false;
        }
    }

    // Intersection criteria: many sensors active, few groups
    return (count >= 4 && groups <= 3);
}

// ========== TURN COMPLETION DETECTION ==========
/*
 * Is_Turn_Complete - Detect when robot has completed a turn
 * Looks for centered line detection after turn maneuver
 */
bool Is_Turn_Complete() {
    int i;
    uint8_t reading = Read_IR_Sensors(CENTER_6_SENSORS);

    // Check for center sensor activation
    bool sensor3 = (reading & 0x08) != 0;
    bool sensor4 = (reading & 0x10) != 0;
    bool center_detected = sensor3 || sensor4;

    if (!center_detected) return false;

    // Count active sensors
    uint8_t count = 0;
    for (i = 1; i < 7; i++) {
        if (reading & (1 << i)) count++;
    }
    if (count > 4) return false;  // Too wide - likely intersection

    // Check for contiguous detection (no gaps in line)
    int first_active = -1, last_active = -1;
    for (i = 1; i < 7; i++) {
        if (reading & (1 << i)) {
            if (first_active == -1) first_active = i;
            last_active = i;
        }
    }

    if (first_active != -1 && last_active != -1) {
        for (i = first_active; i <= last_active; i++) {
            if (!(reading & (1 << i))) return false;
        }
    }

    // Check various alignment patterns for turn completion
    if (sensor3 && sensor4) return true;  // Perfect center
    if ((sensor3 && (reading & 0x04)) || (sensor4 && (reading & 0x20))) return true;  // Good alignment
    if ((sensor3 || sensor4) && count <= 3) return true;  // Acceptable alignment

    return false;
}

// ========== INTERSECTION DIRECTION ANALYSIS ==========
/*
 * Get_Intersection_Direction - Analyze intersection to determine turn direction
 * Compares left vs right sensor activation to decide which way to turn
 */
IntersectionDirection Get_Intersection_Direction(uint8_t reading) {
    int i;
    if (!Is_Intersection(reading)) return INTERSECTION_UNKNOWN;

    // Special case: force right turn at first intersection
    if (first_intersection_after_start) {
        printf("FIRST INTERSECTION AFTER START - FORCING RIGHT TURN\n");
        return INTERSECTION_RIGHT;
    }

    // Count sensors on left side (sensors 4-7) vs right side (sensors 0-3)
    uint8_t left_count = 0, right_count = 0;
    for (i = 0; i < 4; i++) {
        if (reading & (1 << i)) right_count++;
        if (reading & (1 << (i + 4))) left_count++;
    }

    printf("Left sensors: %d, Right sensors: %d\n", left_count, right_count);

    if (left_count > right_count) {
        printf("-> LEFT INTERSECTION DETECTED\n");
        return INTERSECTION_LEFT;
    } else if (right_count > left_count) {
        printf("-> RIGHT INTERSECTION DETECTED\n");
        return INTERSECTION_RIGHT;
    } else {
        printf("-> STRAIGHT INTERSECTION DETECTED\n");
        return INTERSECTION_STRAIGHT;
    }
}

/*
 * Get_Intersection_Direction_From_Backward - Special case for dead-end recovery
 * When arriving at intersection from backward search, prioritize right turns
 */
IntersectionDirection Get_Intersection_Direction_From_Backward(uint8_t reading) {
    int i;
    if (!Is_Intersection(reading)) return INTERSECTION_UNKNOWN;

    uint8_t left_count = 0, right_count = 0;
    for (i = 0; i < 4; i++) {
        if (reading & (1 << i)) right_count++;
        if (reading & (1 << (i + 4))) left_count++;
    }

    printf("BACKWARD ARRIVAL - Left sensors: %d, Right sensors: %d\n", left_count, right_count);

    // Prioritize right turn when multiple options available
    if (right_count > 0 && left_count > 0) {
        printf("-> BOTH DIRECTIONS AVAILABLE - PRIORITIZING RIGHT TURN\n");
        return INTERSECTION_RIGHT;
    } else if (right_count > left_count) {
        printf("-> RIGHT INTERSECTION DETECTED (FROM BACKWARD)\n");
        return INTERSECTION_RIGHT;
    } else if (left_count > right_count) {
        printf("-> LEFT INTERSECTION DETECTED (FROM BACKWARD)\n");
        return INTERSECTION_LEFT;
    } else {
        printf("-> UNCLEAR DIRECTION - DEFAULTING TO RIGHT TURN\n");
        return INTERSECTION_RIGHT;
    }
}

// ========== LINE POSITION DETECTION ==========
/*
 * Get_Line_Position - Main line following algorithm
 * Analyzes sensor pattern to determine where line is relative to robot
 * Returns appropriate steering command for line following
 */
LinePosition Get_Line_Position() {
    uint8_t reading = Read_IR_Sensors(CENTER_6_SENSORS);
    uint8_t all_reading = Read_IR_Sensors(ALL_SENSORS);

    // Check for intersection first
    if (Is_Intersection(all_reading)) return INTERSECTION_DETECTED;

    // Extract individual sensor readings (sensors 1-6)
    bool sensor1 = (reading & 0x02) != 0;
    bool sensor2 = (reading & 0x04) != 0;
    bool sensor3 = (reading & 0x08) != 0;  // Center left
    bool sensor4 = (reading & 0x10) != 0;  // Center right
    bool sensor5 = (reading & 0x20) != 0;
    bool sensor6 = (reading & 0x40) != 0;

    // Determine line position based on active sensors
    if (sensor3 && sensor4) return LINE_CENTERED;        // Perfect center
    else if (sensor3 && !sensor4) return LINE_SLIGHT_RIGHT;  // Slight right correction needed
    else if (sensor4 && !sensor3) return LINE_SLIGHT_LEFT;   // Slight left correction needed
    else if (sensor2) return LINE_SHARP_RIGHT;           // Sharp right correction needed
    else if (sensor5) return LINE_SHARP_LEFT;            // Sharp left correction needed
    else if (sensor1) return LINE_SHARP_RIGHT;           // Very sharp right correction
    else if (sensor6) return LINE_SHARP_LEFT;            // Very sharp left correction
    else return LINE_LOST;                               // No line detected
}

// ========== LOW-LEVEL MOTOR DIRECTION CONTROL ==========
void Left_Forward() { P5->OUT &= ~0x10; }
void Left_Backward() { P5->OUT |= 0x10; }
void Right_Forward() { P5->OUT &= ~0x20; }
void Right_Backward() { P5->OUT |= 0x20; }
void PWM_Duty3(uint16_t duty3) { TIMER_A0->CCR[3] = duty3; }
void PWM_Duty4(uint16_t duty4) { TIMER_A0->CCR[4] = duty4; }

// ========== MOTOR CONTROL FUNCTIONS ==========
/*
 * Movement functions implement proportional steering for line following
 * Faster motor = outer wheel in turn, slower motor = inner wheel
 */
void Move(uint16_t leftDuty, uint16_t rightDuty) {
    P3->OUT |= 0xC0;  // Enable motors
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

void Stop() {
    P3->OUT &= ~0xC0;  // Disable motors
    PWM_Duty3(0);
    PWM_Duty4(0);
}

void Move_Forward() {
    Left_Forward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));
}

void Move_Backward() {
    Left_Backward();
    Right_Backward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));
}

// Forward turning functions (for line following)
void Turn_Slight_Left() {
    Left_Forward();
    Right_Forward();
    Move(Calculate_Speed(TURN_SLOW_SPEED), Calculate_Speed(FULL_SPEED));  // Left slower
}

void Turn_Slight_Right() {
    Left_Forward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(TURN_SLOW_SPEED));  // Right slower
}

void Turn_Sharp_Left() {
    Left_Forward();
    Right_Forward();
    Move(Calculate_Speed(TURN_SHARP_SPEED), Calculate_Speed(FULL_SPEED));  // Left much slower
}

void Turn_Sharp_Right() {
    Left_Forward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(TURN_SHARP_SPEED));  // Right much slower
}

// Backward turning functions (for dead-end recovery)
void Turn_Slight_Left_Backward() {
    Left_Backward();
    Right_Backward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(TURN_SLOW_SPEED));
}

void Turn_Slight_Right_Backward() {
    Left_Backward();
    Right_Backward();
    Move(Calculate_Speed(TURN_SLOW_SPEED), Calculate_Speed(FULL_SPEED));
}

void Turn_Sharp_Left_Backward() {
    Left_Backward();
    Right_Backward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(TURN_SHARP_SPEED));
}

void Turn_Sharp_Right_Backward() {
    Left_Backward();
    Right_Backward();
    Move(Calculate_Speed(TURN_SHARP_SPEED), Calculate_Speed(FULL_SPEED));
}

// ========== INTERSECTION NAVIGATION ==========
/*
 * Execute_Left_Turn - Perform 90-degree left turn at intersection
 * Uses tank turn (one motor forward, one backward) for tight turning
 */
void Execute_Left_Turn() {
    printf("EXECUTING LEFT TURN (FORWARD APPROACH)\n");

    // Move forward slightly to position robot properly
    Move_Forward();
    Clock_Delay1ms(410);
    Stop();
    Clock_Delay1ms(100);

    printf("TURNING LEFT 90 DEGREES");
    // Tank turn: left motor backward, right motor forward
    Left_Backward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));

    // Continue turning until line is detected or safety timeout
    uint32_t safety_counter = 0;
    uint32_t min_turn_time = 0;
    bool turn_complete = false;

    while (min_turn_time < MIN_TURN_TIME) {
        Clock_Delay1ms(50);
        min_turn_time += 50;
        safety_counter++;

        if (min_turn_time >= MIN_TURN_TIME && Is_Turn_Complete()) {
            turn_complete = true;
            break;
        }

        if (safety_counter >= 50) {
            printf("Safety timeout - completing turn\n");
            break;
        }
    }

    if (turn_complete) printf("Turn completed by sensor detection\n");

    Stop();
    Clock_Delay1ms(200);

    // Move forward slightly to ensure robot is on new line
    printf("SEARCHING FOR NEW LINE AFTER FORWARD LEFT TURN\n");
    Move_Forward();
    Clock_Delay1ms(150);
    Stop();
    Clock_Delay1ms(200);
    printf("LEFT TURN COMPLETED\n");
}

/*
 * Execute_Right_Turn - Perform 90-degree right turn at intersection
 * Similar to left turn but with opposite motor directions
 */
void Execute_Right_Turn() {
    printf("EXECUTING RIGHT TURN (FORWARD APPROACH)\n");

    Move_Forward();
    Clock_Delay1ms(410);
    Stop();
    Clock_Delay1ms(100);

    printf("TURNING RIGHT 90 DEGREES");
    // Tank turn: left motor forward, right motor backward
    Left_Forward();
    Right_Backward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));

    uint32_t safety_counter = 0;
    uint32_t min_turn_time = 0;
    bool turn_complete = false;

    while (min_turn_time < MIN_TURN_TIME) {
        Clock_Delay1ms(50);
        min_turn_time += 50;
        safety_counter++;

        if (min_turn_time >= MIN_TURN_TIME && Is_Turn_Complete()) {
            turn_complete = true;
            break;
        }

        if (safety_counter >= 50) {
            printf("Safety timeout - completing turn\n");
            break;
        }
    }

    if (turn_complete) printf("Turn completed by sensor detection\n");

    Stop();
    Clock_Delay1ms(200);

    printf("SEARCHING FOR NEW LINE AFTER FORWARD RIGHT TURN\n");
    Move_Forward();
    Clock_Delay1ms(150);
    Stop();
    Clock_Delay1ms(200);
    printf("RIGHT TURN COMPLETED\n");
}

// Turn functions for dead-end recovery (when approaching intersection from backward)
void Execute_Left_Turn_From_Backward() {
    printf("EXECUTING LEFT TURN (BACKWARD APPROACH)\n");

    Move_Forward();
    Clock_Delay1ms(200);
    Stop();
    Clock_Delay1ms(200);

    printf("TURNING LEFT 90 DEGREES AFTER BACKING UP");
    Left_Backward();
    Right_Forward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));

    uint32_t safety_counter = 0;
    uint32_t min_turn_time = 0;
    bool turn_complete = false;

    while (min_turn_time < MIN_TURN_TIME) {
        Clock_Delay1ms(50);
        min_turn_time += 50;
        safety_counter++;

        if (min_turn_time >= MIN_TURN_TIME && Is_Turn_Complete()) {
            turn_complete = true;
            break;
        }

        if (safety_counter >= 50) {
            printf("Safety timeout - completing turn\n");
            break;
        }
    }

    if (turn_complete) printf("Turn completed by sensor detection\n");

    Stop();
    Clock_Delay1ms(200);
    printf("LEFT TURN FROM BACKWARD COMPLETED\n");
}

void Execute_Right_Turn_From_Backward() {
    printf("EXECUTING RIGHT TURN (BACKWARD APPROACH)\n");

    Move_Forward();
    Clock_Delay1ms(200);
    Stop();
    Clock_Delay1ms(200);

    printf("TURNING RIGHT 90 DEGREES AFTER BACKING UP");
    Left_Forward();
    Right_Backward();
    Move(Calculate_Speed(FULL_SPEED), Calculate_Speed(FULL_SPEED));

    uint32_t safety_counter = 0;
    uint32_t min_turn_time = 0;
    bool turn_complete = false;

    while (min_turn_time < MIN_TURN_TIME) {
        Clock_Delay1ms(50);
        min_turn_time += 50;
        safety_counter++;

        if (min_turn_time >= MIN_TURN_TIME && Is_Turn_Complete()) {
            turn_complete = true;
            break;
        }

        if (safety_counter >= 50) {
            printf("Safety timeout - completing turn\n");
            break;
        }
    }

    if (turn_complete) printf("Turn completed by sensor detection\n");

    Stop();
    Clock_Delay1ms(200);
    printf("RIGHT TURN FROM BACKWARD COMPLETED\n");
}

// ========== START DETECTION ==========
/*
 * Check_Start_Line - Detect when robot is placed on starting line
 * Uses middle 4 sensors to detect wide line pattern
 */
bool Check_Start_Line() {
    uint8_t reading = Read_IR_Sensors(MIDDLE_4_SENSORS);
    return (reading & MIDDLE_4_SENSORS) == MIDDLE_4_SENSORS;
}

// ========== MAIN PROGRAM ==========
/*
 * MAIN STATE MACHINE
 * Controls overall robot behavior through different states:
 * 1. WAITING - Wait for start signal
 * 2. PLACEMENT_WAIT - 5-second countdown for user adjustment
 * 3. INITIAL_FORWARD - Move forward off starting line
 * 4. FOLLOWING - Main line following mode
 * 5. STOPPED - Analyze intersection
 * 6. LINE_LOST_WAIT - Stabilization after line loss
 * 7. BACKWARD_SEARCH - Search for lost line by going backward
 * 8. BACKWARD_TO_INTERSECTION - Navigate backward to intersection
 */
int main(void) {
    // Initialize hardware systems
    Clock_Init48MHz();
    Motor_Init();
    IR_Init();
    Set_Robot_Speed(DEFAULT_ROBOT_SPEED);

    // Initialize state machine variables
    RobotState current_state = ROBOT_WAITING;
    uint8_t start_counter = 0;
    static uint32_t blink_counter = 0;
    bool arrived_from_backward = false;  // Flag for dead-end recovery
    uint32_t placement_timer = 0;
    uint32_t forward_timer = 0;
    uint32_t line_lost_timer = 0;

    LED_Red_On();  // Initial state indicator

    while (1) {
        switch (current_state) {

            case ROBOT_WAITING:
                // Wait for start signal from middle 4 sensors detecting wide line
                if (Check_Start_Line()) {
                    start_counter++;
                    if (start_counter >= 3) {
                        printf("FOUR MIDDLE SENSORS DETECTED - STARTING 5 SECOND PLACEMENT WAIT\n");
                        current_state = ROBOT_PLACEMENT_WAIT;
                        LED_Magenta_On();
                        placement_timer = 0;
                        start_counter = 0;
                        arrived_from_backward = false;
                        first_intersection_after_start = true;
                        intersection_count = 0;
                    }
                } else {
                    start_counter = 0;
                    // Blink red LED to show robot is alive and waiting
                    blink_counter++;
                    if (blink_counter % 20 == 0) {
                        LED_Off();
                        Clock_Delay1ms(100);
                        LED_Red_On();
                    }
                }
                break;

            case ROBOT_PLACEMENT_WAIT:
                // 5-second countdown allows user to adjust robot placement
                LED_Magenta_On();
                placement_timer++;

                if (placement_timer % 20 == 0) {
                    uint8_t seconds_remaining = 5 - (placement_timer / 20);
                    if (seconds_remaining > 0) {
                        printf("PLACEMENT ADJUSTMENT TIME REMAINING: %d seconds\n", seconds_remaining);
                    }
                }

                if (placement_timer >= 100) {  // 5 seconds at 50ms intervals
                    printf("PLACEMENT WAIT COMPLETE - STARTING INITIAL FORWARD MOVEMENT\n");
                    current_state = ROBOT_INITIAL_FORWARD;
                    LED_White_On();
                    forward_timer = 0;
                }
                break;

            case ROBOT_INITIAL_FORWARD:
                // Move forward to clear starting line before beginning line following
                LED_White_On();
                Move_Forward();
                forward_timer++;

                if (forward_timer >= 15) {  // 750ms at 50ms intervals
                    printf("INITIAL FORWARD COMPLETE - STARTING LINE FOLLOWING NAVIGATION\n");
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    forward_timer = 0;
                }
                break;

            case ROBOT_FOLLOWING:
                // Main line following mode - continuously adjust steering based on line position
                LED_Blue_On();
                LinePosition line_pos = Get_Line_Position();

                // Debug output every 10 cycles (500ms)
                static uint8_t debug_counter = 0;
                debug_counter++;
                if (debug_counter % 10 == 0) {
                    uint8_t all_sensors = Read_IR_Sensors(ALL_SENSORS);
                    Debug_Sensor_Pattern(all_sensors);
                }

                // Execute appropriate movement based on line position
                switch (line_pos) {
                    case INTERSECTION_DETECTED:
                        printf("INTERSECTION DETECTED - STOPPING\n");
                        Stop();
                        current_state = ROBOT_STOPPED;
                        LED_Green_On();
                        break;
                    case LINE_CENTERED: Move_Forward(); break;
                    case LINE_SLIGHT_LEFT: Turn_Slight_Left(); break;
                    case LINE_SLIGHT_RIGHT: Turn_Slight_Right(); break;
                    case LINE_SHARP_LEFT: Turn_Sharp_Left(); break;
                    case LINE_SHARP_RIGHT: Turn_Sharp_Right(); break;
                    case LINE_LOST:
                        printf("LINE LOST - STARTING 300ms STABILIZATION WAIT\n");
                        Stop();
                        current_state = ROBOT_LINE_LOST_WAIT;
                        LED_Orange_On();
                        line_lost_timer = 0;
                        break;
                    default: Move_Forward(); break;
                }
                break;

            case ROBOT_LINE_LOST_WAIT:
                // Brief wait after line loss to avoid false alarms from shadows/noise
                LED_Orange_On();
                Stop();
                line_lost_timer++;

                // Check if line is found again during wait period
                LinePosition wait_line_pos = Get_Line_Position();
                if (wait_line_pos != LINE_LOST) {
                    printf("LINE FOUND AGAIN DURING WAIT - RESUMING FOLLOWING\n");
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    line_lost_timer = 0;
                    break;
                }

                if (line_lost_timer >= 6) {  // 300ms at 50ms intervals
                    printf("300ms STABILIZATION COMPLETE - STARTING BACKWARD SEARCH\n");
                    current_state = ROBOT_BACKWARD_SEARCH;
                    LED_Yellow_On();
                    line_lost_timer = 0;
                }
                break;

            case ROBOT_BACKWARD_SEARCH:
                // Search for lost line by moving backward (dead-end recovery)
                LED_Yellow_On();
                LinePosition backward_line_pos = Get_Line_Position();

                switch (backward_line_pos) {
                    case LINE_LOST:
                        Move_Backward();
                        printf("SEARCHING BACKWARD FOR LOST LINE...\n");
                        break;
                    case INTERSECTION_DETECTED:
                        printf("INTERSECTION FOUND DURING BACKWARD SEARCH\n");
                        Stop();
                        current_state = ROBOT_STOPPED;
                        LED_Green_On();
                        arrived_from_backward = true;
                        break;
                    default:
                        printf("LINE FOUND - CONTINUING BACKWARD TO INTERSECTION\n");
                        current_state = ROBOT_BACKWARD_TO_INTERSECTION;
                        LED_Cyan_On();
                        break;
                }
                break;

            case ROBOT_BACKWARD_TO_INTERSECTION:
                // Follow line backward until reaching intersection for dead-end recovery
                LED_Cyan_On();
                LinePosition backward_to_int_pos = Get_Line_Position();

                switch (backward_to_int_pos) {
                    case INTERSECTION_DETECTED:
                        printf("INTERSECTION REACHED WHILE GOING BACKWARD\n");
                        Stop();
                        current_state = ROBOT_STOPPED;
                        LED_Green_On();
                        arrived_from_backward = true;
                        break;
                    case LINE_CENTERED: Move_Backward(); break;
                    case LINE_SLIGHT_LEFT: Turn_Slight_Left_Backward(); break;
                    case LINE_SLIGHT_RIGHT: Turn_Slight_Right_Backward(); break;
                    case LINE_SHARP_LEFT: Turn_Sharp_Left_Backward(); break;
                    case LINE_SHARP_RIGHT: Turn_Sharp_Right_Backward(); break;
                    case LINE_LOST:
                        printf("LINE LOST AGAIN - RETURNING TO BACKWARD SEARCH\n");
                        current_state = ROBOT_BACKWARD_SEARCH;
                        LED_Yellow_On();
                        break;
                    default: Move_Backward(); break;
                }
                break;

            case ROBOT_STOPPED:
                // Analyze intersection and decide which direction to turn
                LED_Green_On();
                Clock_Delay1ms(INTERSECTION_PAUSE_TIME);  // Pause for analysis

                if (arrived_from_backward) {
                    // Special handling for dead-end recovery
                    printf("ARRIVED FROM BACKWARD - EXECUTING DEAD-END RECOVERY\n");

                    // Take multiple sensor readings for reliable analysis
                    uint8_t readings[3];
                    int i, r;
                    for (i = 0; i < 3; i++) {
                        readings[i] = Read_IR_Sensors(ALL_SENSORS);
                        if (i < 2) Clock_Delay1ms(100);
                    }

                    // Use reading with most sensors active for best accuracy
                    uint8_t final_reading = readings[0];
                    uint8_t max_count = 0;
                    for (r = 0; r < 3; r++) {
                        uint8_t count = 0;
                        for (i = 0; i < 8; i++) {
                            if (readings[r] & (1 << i)) count++;
                        }
                        if (count > max_count) {
                            max_count = count;
                            final_reading = readings[r];
                        }
                    }

                    printf("BACKWARD ARRIVAL INTERSECTION READING: ");
                    Debug_Sensor_Pattern(final_reading);

                    IntersectionDirection direction = Get_Intersection_Direction_From_Backward(final_reading);

                    switch (direction) {
                        case INTERSECTION_LEFT:
                            printf("EXECUTING LEFT TURN FROM BACKWARD POSITION\n");
                            Execute_Left_Turn_From_Backward();
                            break;
                        case INTERSECTION_RIGHT:
                        default:
                            printf("EXECUTING RIGHT TURN FROM BACKWARD POSITION\n");
                            Execute_Right_Turn_From_Backward();
                            break;
                    }

                    arrived_from_backward = false;
                    current_state = ROBOT_FOLLOWING;
                    LED_Blue_On();
                    printf("RESUMING FORWARD LINE FOLLOWING AFTER DEAD-END RECOVERY\n");

                } else {
                    // Normal intersection handling (forward approach)
                    uint8_t all_sensors = Read_IR_Sensors(ALL_SENSORS);
                    printf("STOPPED: ");
                    Debug_Sensor_Pattern(all_sensors);

                    // Take multiple readings for reliability
                    uint8_t readings[3];
                    int i, r;
                    for (i = 0; i < 3; i++) {
                        readings[i] = Read_IR_Sensors(ALL_SENSORS);
                        if (i < 2) Clock_Delay1ms(100);
                    }

                    // Use reading with most sensors active
                    uint8_t final_reading = readings[0];
                    uint8_t max_count = 0;
                    for (r = 0; r < 3; r++) {
                        uint8_t count = 0;
                        for (i = 0; i < 8; i++) {
                            if (readings[r] & (1 << i)) count++;
                        }
                        if (count > max_count) {
                            max_count = count;
                            final_reading = readings[r];
                        }
                    }

                    printf("FINAL INTERSECTION READING: ");
                    Debug_Sensor_Pattern(final_reading);

                    if (Is_Intersection(final_reading)) {
                        intersection_count++;
                        printf("INTERSECTION #%d DETECTED\n", intersection_count);

                        IntersectionDirection direction = Get_Intersection_Direction(final_reading);

                        switch (direction) {
                            case INTERSECTION_LEFT:
                                printf("CONFIRMED LEFT INTERSECTION - EXECUTING FORWARD LEFT TURN\n");
                                Execute_Left_Turn();
                                break;
                            case INTERSECTION_RIGHT:
                                printf("CONFIRMED RIGHT INTERSECTION - EXECUTING FORWARD RIGHT TURN\n");
                                Execute_Right_Turn();
                                break;
                            case INTERSECTION_STRAIGHT:
                                printf("STRAIGHT INTERSECTION - MOVING FORWARD\n");
                                Move_Forward();
                                Clock_Delay1ms(500);
                                break;
                            default:
                                printf("UNKNOWN INTERSECTION TYPE - DEFAULTING TO FORWARD\n");
                                Move_Forward();
                                Clock_Delay1ms(300);
                                break;
                        }

                        if (first_intersection_after_start) {
                            first_intersection_after_start = false;
                            printf("FIRST INTERSECTION HANDLED - NORMAL DETECTION RESUMED\n");
                        }

                        current_state = ROBOT_FOLLOWING;
                        LED_Blue_On();
                        printf("RESUMING LINE FOLLOWING AFTER INTERSECTION\n");
                    } else {
                        printf("NO INTERSECTION DETECTED - RESUMING LINE FOLLOWING\n");
                        current_state = ROBOT_FOLLOWING;
                        LED_Blue_On();
                    }
                }
                break;
        }

        Clock_Delay1ms(50);  // 50ms main loop cycle for responsive control
    }
}
