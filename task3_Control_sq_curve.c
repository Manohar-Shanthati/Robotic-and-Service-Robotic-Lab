/*******************************************************************************
 * Robot Motion Control Program for Khepera4
 * 
 * This program implements various motion control functions for the Khepera4 robot:
 * - Straight line movement (at different speeds)
 * - Curved path following (clockwise and counterclockwise)
 * - 90-degree turns (left and right)
 * 
 * The program uses encoders for position tracking and IR sensors for obstacle
 * detection. It implements differential drive kinematics for precise motion
 * control and includes safety features to prevent collisions.
 ******************************************************************************/

#include <khepera/khepera.h>  // Khepera4 robot control library
#include <signal.h>           // For handling interrupt signals
#include <stdio.h>           // Standard I/O functions
#include <math.h>            // Mathematical functions (for M_PI)

// Global variables for position tracking
int lpos, rpos;              // Current encoder positions
int lpos_prev, rpos_prev;    // Previous encoder positions
double fpos, dval;           // Forward position and delta value
struct timeval startt, endt, lastPrintTime;  // Timing variables
static knet_dev_t *dsPic;    // Robot's dsPic microcontroller handle
static int quitReq = 0;      // Program termination flag

// Robot physical parameters (in millimeters)
#define KH4_WHEELBASE 105.4  // Distance between wheels
#define WHEEL_RADIUS 21.0    // Wheel radius
#define M_PI 3.14159265358979323846  // Mathematical constant pi

char Buffer[100];            // Buffer for sensor readings
char bar[12][64];           // Display buffer
char revision, version;      // Robot firmware version info

// IR sensor configuration
#define FRONT_IR_SENSOR_INDEX 3        // Front sensor index
#define FRONT_LEFT_IR_SENSOR_INDEX 2   // Front-left sensor index
#define FRONT_RIGHT_IR_SENSOR_INDEX 4  // Front-right sensor index
#define IR_THRESHOLD 800               // Obstacle detection threshold

/*******************************************************************************
 * Calculate time difference between two timevals
 * 
 * @param difference: Where to store the result (can be NULL)
 * @param end_time: End time
 * @param start_time: Start time
 * @return: Time difference in microseconds
 ******************************************************************************/
long long timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time) {
    struct timeval temp_diff;

    if (difference == NULL) {
        difference = &temp_diff;
    }

    // Calculate seconds and microseconds difference
    difference->tv_sec = end_time->tv_sec - start_time->tv_sec;
    difference->tv_usec = end_time->tv_usec - start_time->tv_usec;

    // Handle microsecond underflow
    while (difference->tv_usec < 0) {
        difference->tv_usec += 1000000;
        difference->tv_sec -= 1;
    }

    return 1000000LL * difference->tv_sec + difference->tv_usec;
}

// Time structure definition for high-precision timing
struct timeval {
    time_t      tv_sec;     /* seconds */
    suseconds_t tv_usec;    /* microseconds */
};

/*******************************************************************************
 * Handler for Ctrl+C interrupt
 * Performs clean shutdown of robot when program is interrupted
 ******************************************************************************/
static void ctrlc_handler(int sig) {
    quitReq = 1;
    kh4_set_speed(0, 0, dsPic);         // Stop robot movement
    kh4_SetMode(kh4RegIdle, dsPic);     // Set motors to idle
    kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic);  // Turn off LEDs
    kb_change_term_mode(0);              // Restore terminal settings
    exit(0);
}

/*******************************************************************************
 * Check if any IR sensors detect obstacles
 * 
 * @param frontIRReading: Reading from front sensor
 * @param frontLeftIRReading: Reading from front-left sensor
 * @param frontRightIRReading: Reading from front-right sensor
 * @return: 1 if obstacle detected, 0 otherwise
 ******************************************************************************/
int areIRSensorsTriggered(int frontIRReading, int frontLeftIRReading, int frontRightIRReading) {
    return frontIRReading > IR_THRESHOLD || 
           frontLeftIRReading > IR_THRESHOLD || 
           frontRightIRReading > IR_THRESHOLD;
}

/*******************************************************************************
 * Monitor IR sensors and stop robot if obstacle detected
 * 
 * @param dsPic: Robot controller handle
 ******************************************************************************/
void checkIRSensorsAndStop(knet_dev_t *dsPic) {
    kh4_proximity_ir(Buffer, dsPic);
    
    // Read IR sensor values (combine two 8-bit values into 16-bit)
    int frontIRReading = (Buffer[FRONT_IR_SENSOR_INDEX * 2] | 
                         Buffer[FRONT_IR_SENSOR_INDEX * 2 + 1] << 8);
    int frontLeftIRReading = (Buffer[FRONT_LEFT_IR_SENSOR_INDEX * 2] | 
                            Buffer[FRONT_LEFT_IR_SENSOR_INDEX * 2 + 1] << 8);
    int frontRightIRReading = (Buffer[FRONT_RIGHT_IR_SENSOR_INDEX * 2] | 
                             Buffer[FRONT_RIGHT_IR_SENSOR_INDEX * 2 + 1] << 8);

    if (areIRSensorsTriggered(frontIRReading, frontLeftIRReading, frontRightIRReading)) {
        printf("One of the IR sensors triggered. Stopping the robot.\n");
        kh4_set_speed(0, 0, dsPic);
        quitReq = 1;
    }
}

/*******************************************************************************
 * Move robot in straight line for specified distance
 * 
 * @param distanceToTravel: Target distance in millimeters
 * Uses encoder feedback for precise distance control
 ******************************************************************************/
void straight_fn(int distanceToTravel) {
    kh4_ResetEncoders(dsPic);
    double distanceTraveled = 0.0;
    double distance = 0.0;
    gettimeofday(&startt, NULL);
    int lpos_prev = 0, rpos_prev = 0;

    // Set constant speed for straight motion
    kh4_SetMode(kh4RegSpeed, dsPic);
    kh4_set_speed(100, 100, dsPic);

    while (distanceTraveled < distanceToTravel && !quitReq) {
        kh4_get_position(&lpos, &rpos, dsPic);

        // Calculate wheel velocities from encoder differences
        double vl = (lpos - lpos_prev) * KH4_PULSE_TO_MM;
        double vr = (rpos - rpos_prev) * KH4_PULSE_TO_MM;

        // Update distance using average of wheel movements
        double distanceIncrement = ((vl + vr) / 2.0);
        distanceTraveled += distanceIncrement;
        
        gettimeofday(&endt, NULL);
        checkIRSensorsAndStop(dsPic);

        // Progress update every 50ms
        if (timeval_diff(NULL, &endt, &lastPrintTime) >= 50000) {
            printf("Distance traveled: %.2f mm\n", distanceTraveled);
            gettimeofday(&lastPrintTime, NULL);
        }

        lpos_prev = lpos;
        rpos_prev = rpos;
        usleep(10000);  // 10ms control loop interval
    }

    // Stop and calculate final distance
    kh4_set_speed(0, 0, dsPic);
    kh4_get_position(&lpos, &rpos, dsPic);
    
    double vl = (lpos) * KH4_PULSE_TO_MM;
    double vr = (rpos) * KH4_PULSE_TO_MM;
    distance = ((vl + vr) / 2.0);
    
    printf("Total Distance travelled in straight line: %.2f mm\n", distance);
}

// High-speed straight line movement function (speed 400 instead of 100)
void straight_fn_400(int distanceToTravel) {
    // Implementation identical to straight_fn but with speed 400
    // Code omitted for brevity
}

/*******************************************************************************
 * Move robot in counterclockwise curved path
 * 
 * @param R: Radius of curve in millimeters
 * @param omega_degrees: Angular distance to travel in degrees
 * Uses differential drive equations for curved motion
 ******************************************************************************/
void curve_anticlockwise_fn(int R, int omega_degrees) {
    // Convert degrees to radians for calculations
    double omega = omega_degrees * (M_PI / 180.0);
    kh4_ResetEncoders(dsPic);

    // Calculate arc length to travel
    double distanceToTravel = omega * R;
    printf("Distance to travel: %.2f mm\n", distanceToTravel);
    double distanceThreshold = 0.995 * distanceToTravel;  // 99.5% completion threshold
    double distance = 0.0;
    
    gettimeofday(&startt, NULL);
    gettimeofday(&lastPrintTime, NULL);

    while (!quitReq) {
        // Calculate differential drive velocities for curved path
        double vr = (R + KH4_WHEELBASE / 2);  // Outer wheel
        double vl = (R - KH4_WHEELBASE / 2);  // Inner wheel

        kh4_SetMode(kh4RegSpeed, dsPic);
        kh4_set_speed((int)vl, (int)vr, dsPic);

        // Progress update every 100ms
        if (timeval_diff(NULL, &endt, &lastPrintTime) >= 100000) {
            kh4_get_position(&lpos, &rpos, dsPic);
            
            // Calculate distance traveled
            double vl = (lpos) * KH4_PULSE_TO_MM;
            double vr = (rpos) * KH4_PULSE_TO_MM;
            distance = ((vl + vr) / 2.0);
            
            printf("Distance travelled: %.2f mm\n", distance);

            // Check if target distance reached
            if (distance >= distanceThreshold) {
                printf("Distance reached the threshold. Stopping the robot.\n");
                kh4_set_speed(0, 0, dsPic);
                
                // Final distance calculation
                kh4_get_position(&lpos, &rpos, dsPic);
                vl = (lpos) * KH4_PULSE_TO_MM;
                vr = (rpos) * KH4_PULSE_TO_MM;
                distance = ((vl + vr) / 2.0);
                
                printf("Total Distance travelled: %.2f mm\n", distance);
                break;
            }

            gettimeofday(&lastPrintTime, NULL);
        }

        usleep(10000);  // 10ms control loop interval
        gettimeofday(&endt, NULL);
        checkIRSensorsAndStop(dsPic);
    }
}

/*******************************************************************************
 * Move robot in clockwise curved path
 * 
 * @param R: Radius of curve in millimeters
 * @param omega_degrees: Angular distance to travel in degrees
 * Similar to anticlockwise function but with reversed wheel velocities
 ******************************************************************************/
void curve_clockwise_fn(int R, int omega_degrees) {
    // Implementation similar to curve_anticlockwise_fn
    // but with vr and vl calculations swapped
    // Code structure identical, only velocity calculations differ
    // Code omitted for brevity as it's a mirror of curve_anticlockwise_fn
}

/*******************************************************************************
 * Perform quick 90-degree left turn
 * Uses timed differential wheel speeds for rotation
 ******************************************************************************/
void turnLeft90Degrees() {
    int duration = 4000000;  // 4 second turn duration
    int speedLeft = -20;     // Left wheel reverses
    int speedRight = 40;     // Right wheel forward
    
    kh4_SetMode(kh4RegSpeed, dsPic);
    kh4_set_speed(speedLeft, speedRight, dsPic);
    usleep(duration);
    kh4_set_speed(0, 0, dsPic);
}

/*******************************************************************************
 * Perform quick 90-degree right turn
 * Mirror of left turn with reversed wheel speeds
 ******************************************************************************/
void turnRight90Degrees() {
    int duration = 4000000;  // 4 second turn duration
    int speedLeft = 40;      // Left wheel forward
    int speedRight = -20;    // Right wheel reverses
    
    kh4_SetMode(kh4RegSpeed, dsPic);
    kh4_set_speed(speedLeft, speedRight, dsPic);
    usleep(duration);
    kh4_set_speed(0, 0, dsPic);
}

/*******************************************************************************
 * Main program
 * Initializes robot and demonstrates various movement functions
 ******************************************************************************/
int main() {
    // Initialize libkhepera
    if (kh4_init(0, NULL) != 0) {
        printf("\nERROR: could not initiate the libkhepera!\n\n");
        return -1;
    }

    // Connect to robot's microcontroller
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
    if (dsPic == NULL) {
        printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
        return -2;
    }

    // Set up interrupt handler and reset encoders
    signal(SIGINT, ctrlc_handler);
    kh4_ResetEncoders(dsPic);

    // Example movement sequence
    straight_fn(200);              // Move forward 200mm at normal speed
    usleep(100000);               // Brief pause between movements
    straight_fn_400(200);         // Move forward 200mm at high speed
    usleep(100000);
    curve_fn(200, 180);          // Follow curved path
    usleep(100000);
    turnRight90Degrees();         // Quick right turn
    usleep(100000);
    turnLeft90Degrees();          // Quick left turn
    
    // Cleanup and exit
    kh4_set_speed(0, 0, dsPic);           // Stop movement
    kh4_SetMode(kh4RegIdle, dsPic);       // Idle motors
    knet_close(dsPic);                    // Close communication
    kb_change_term_mode(0);               // Reset terminal

    return 0;
}
