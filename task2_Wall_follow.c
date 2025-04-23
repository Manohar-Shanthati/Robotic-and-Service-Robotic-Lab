/*******************************************************************************
 * Wall Following Program for Khepera4 Robot
 * 
 * This program implements a wall following algorithm that allows the robot to
 * navigate alongside walls while maintaining a safe distance. It uses three IR
 * sensors to detect walls and obstacles:
 * - Right sensor: Maintains proper distance from wall
 * - Front-right sensor: Helps detect approaching corners
 * - Front sensor: Detects obstacles directly ahead
 * 
 * The robot adjusts its movement based on sensor readings to maintain a desired
 * distance from the wall and avoid collisions.
 ******************************************************************************/

#include <khepera/khepera.h>  // Main Khepera4 robot control library
#include <signal.h>           // For handling interrupt signals (Ctrl+C)
#include <stdio.h>           // Standard I/O operations

static knet_dev_t *dsPic;    // Robot's dsPic microcontroller access handle
static int quitReq = 0;      // Control flag for program termination
char Buffer[100];            // Buffer for storing sensor readings

// IR sensor indices for wall detection
#define RIGHT_IR_SENSOR 5        // Right-side sensor for wall distance measurement
#define FRONT_RIGHT_IR_SENSOR 4  // Front-right sensor for corner detection
#define FRONT_IR_SENSOR 3        // Front sensor for obstacle detection

// Distance thresholds for wall following behavior (in IR sensor units)
#define DESIRED_DISTANCE 400     // Target distance to maintain from wall
#define TOO_CLOSE 600           // Threshold indicating dangerously close to wall
#define TOO_FAR 200             // Threshold indicating too far from wall

// Speed settings for robot movement (in motor units)
#define BASE_SPEED 150          // Default movement speed
#define MAX_SPEED 250           // Maximum allowed speed
#define MIN_SPEED 50           // Minimum speed for stable movement

/*******************************************************************************
 * Interrupt handler for Ctrl+C
 * Performs clean shutdown of robot when program is interrupted:
 * - Stops robot movement
 * - Sets motors to idle mode
 * - Turns off RGB LEDs to save power
 * - Restores terminal settings
 ******************************************************************************/
static void ctrlc_handler(int sig) {
    quitReq = 1;
    kh4_set_speed(0, 0, dsPic);                // Stop robot movement
    kh4_SetMode(kh4RegIdle, dsPic);           // Set motors to idle
    kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, dsPic);  // Turn off all RGB LEDs
    kb_change_term_mode(0);                   // Restore terminal settings
    exit(0);
}

/*******************************************************************************
 * Reads value from specified IR sensor
 * 
 * @param sensor_index: Index of the IR sensor to read
 * @return: 16-bit sensor reading (higher values indicate closer obstacles)
 ******************************************************************************/
int read_ir_sensor(int sensor_index) {
    kh4_proximity_ir(Buffer, dsPic);  // Read all IR sensors
    // Combine two 8-bit values into 16-bit sensor reading
    return (Buffer[sensor_index * 2] | Buffer[sensor_index * 2 + 1] << 8);
}

/*******************************************************************************
 * Main wall following function
 * Implements the wall following algorithm using IR sensors:
 * - Continuously monitors wall distance using right sensor
 * - Detects obstacles and corners using front sensors
 * - Adjusts motor speeds to maintain desired wall distance
 * - Includes safety checks to prevent collisions
 ******************************************************************************/
void wall_following() {
    int right_sensor, front_right_sensor, front_sensor;  // IR sensor values
    int left_speed, right_speed;                        // Motor speeds

    // Enable speed control mode for smooth movement
    kh4_SetMode(kh4RegSpeed, dsPic);

    while (!quitReq) {
        // Read values from all relevant sensors
        right_sensor = read_ir_sensor(RIGHT_IR_SENSOR);
        front_right_sensor = read_ir_sensor(FRONT_RIGHT_IR_SENSOR);
        front_sensor = read_ir_sensor(FRONT_IR_SENSOR);

        // Print sensor values for debugging and monitoring
        printf("Right: %d, Front-Right: %d, Front: %d\n", 
               right_sensor, front_right_sensor, front_sensor);

        // Initialize speeds to base values
        left_speed = BASE_SPEED;
        right_speed = BASE_SPEED;

        // Primary obstacle avoidance - check front sensor first
        if (front_sensor > TOO_CLOSE) {
            // Obstacle detected ahead - execute sharp left turn
            left_speed = -MIN_SPEED;    // Reverse left motor for sharp turn
            right_speed = MAX_SPEED;    // Maximum right motor speed
            printf("Turning left - obstacle ahead\n");
        }
        // Wall following behavior when no front obstacle
        else {
            if (right_sensor > TOO_CLOSE) {
                // Too close to wall - adjust by turning left
                left_speed = MIN_SPEED;
                right_speed = MAX_SPEED;
                printf("Too close to wall - adjusting left\n");
            }
            else if (right_sensor < TOO_FAR) {
                // Too far from wall - adjust by turning right
                left_speed = MAX_SPEED;
                right_speed = MIN_SPEED;
                printf("Too far from wall - adjusting right\n");
            }
            else {
                // Within acceptable distance range - fine-tune position
                // Calculate error from desired distance
                int error = right_sensor - DESIRED_DISTANCE;
                // Apply proportional control adjustment
                int adjustment = error / 10;  // Divide by 10 to dampen response

                // Adjust speeds based on error
                left_speed = BASE_SPEED - adjustment;
                right_speed = BASE_SPEED + adjustment;

                // Ensure speeds remain within safe bounds
                left_speed = left_speed > MAX_SPEED ? MAX_SPEED : 
                            (left_speed < MIN_SPEED ? MIN_SPEED : left_speed);
                right_speed = right_speed > MAX_SPEED ? MAX_SPEED : 
                             (right_speed < MIN_SPEED ? MIN_SPEED : right_speed);

                printf("Maintaining distance - L:%d R:%d\n", left_speed, right_speed);
            }
        }

        // Apply the calculated speeds to motors
        kh4_set_speed(left_speed, right_speed, dsPic);

        // Brief delay to prevent sensor reading overflow
        usleep(50000);  // 50ms delay for system stability
    }
}

/*******************************************************************************
 * Main program entry point
 * Initializes robot communication and starts wall following:
 * - Sets up libkhepera and establishes robot connection
 * - Configures interrupt handler
 * - Runs wall following algorithm
 * - Performs cleanup on exit
 ******************************************************************************/
int main() {
    printf("\nKhepera IV Wall Following Program\n");

    // Initialize the libkhepera library
    if (kh4_init(0, NULL) != 0) {
        printf("\nERROR: could not initiate the libkhepera!\n\n");
        return -1;
    }

    // Establish communication with robot's microcontroller
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
    if (dsPic == NULL) {
        printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
        return -2;
    }

    // Configure interrupt handler for clean program termination
    signal(SIGINT, ctrlc_handler);

    // Start the wall following behavior
    printf("Starting wall following behavior...\n");
    printf("Press Ctrl-C to stop\n");
    wall_following();

    // Perform cleanup operations
    kh4_set_speed(0, 0, dsPic);           // Stop robot movement
    kh4_SetMode(kh4RegIdle, dsPic);      // Set motors to idle
    knet_close(dsPic);                   // Close communication
    kb_change_term_mode(0);              // Restore terminal settings

    return 0;
}
