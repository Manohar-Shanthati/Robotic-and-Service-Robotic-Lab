/*******************************************************************************
 * Line Following Program for Khepera4 Robot
 * 
 * This program implements a line following algorithm using IR sensors to detect
 * and follow a black line on the ground. The robot adjusts its movement based
 * on sensor readings to stay on the line path.
 ******************************************************************************/

#include <khepera/khepera.h>  // Main Khepera4 robot control library
#include <signal.h>           // For handling interrupt signals (Ctrl+C)
#include <stdio.h>           // Standard I/O operations

// Global robot control handle
static knet_dev_t * dsPic;   // Robot's dsPic microcontroller access handle

// Control flag for main loop
static int quitReq = 0;      // Set to 1 when program should terminate

// Speed and control constants for line following behavior
#define LF_fwSpeed 90        // Normal forward speed when moving straight
#define LF_ROT_DIFF_SPEED 50 // Speed difference applied during turns
#define LF_ROT_SPEED 70      // Base rotation speed for turns
#define LF_LINE_THRESH 800   // IR threshold for detecting black line (values below this indicate black)
#define LF_ROT_SPEED_AROUND 60 // Speed for rotating when searching for line
#define LF_LEFT 1            // Direction indicator for left turns
#define LF_RIGHT 0           // Direction indicator for right turns

/*******************************************************************************
 * Interrupt handler for Ctrl+C
 * Performs clean shutdown of robot when program is interrupted:
 * - Stops robot movement
 * - Sets motors to idle mode
 * - Turns off RGB LEDs to save power
 * - Restores terminal settings
 ******************************************************************************/
static void ctrlc_handler(int sig)
{
  quitReq = 1;
  kh4_set_speed(0, 0, dsPic);                              // Stop robot movement
  kh4_SetMode(kh4RegIdle, dsPic);                         // Set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic);              // Turn off all RGB LEDs
  kb_change_term_mode(0);                                 // Restore terminal settings
  exit(0);
}

/*******************************************************************************
 * Main line following function
 * Implements the line following algorithm using IR sensors:
 * - Continuously reads IR sensors to detect line position
 * - Adjusts motor speeds based on sensor readings
 * - Handles different scenarios: straight line, turns, line recovery
 ******************************************************************************/
void line_following()
{
  int left_ir, right_ir;           // IR sensor values
  int lspeed, rspeed;             // Left and right motor speeds
  int last_dir = LF_RIGHT;        // Last turn direction (used for line recovery)

  char Buffer[256];               // Buffer for sensor readings

  kh4_SetMode(kh4RegSpeedProfile, dsPic);  // Enable speed profile control mode

  printf("\nPush ANY KEY to stop!\n");

  while(!kb_kbhit())  // Main control loop - runs until key press
  {
    lspeed = 0; rspeed = 0;

    // Read IR sensor values
    kh4_proximity_ir(Buffer, dsPic);
    left_ir = (Buffer[9*2] | Buffer[9*2+1]<<8);   // Left sensor reading
    right_ir = (Buffer[10*2] | Buffer[10*2+1]<<8); // Right sensor reading
    // print left_ir and right_ir readings to the screen
		printf("left_ir: %d, right_ir: %d\n", left_ir, right_ir);

    // Line following logic based on sensor readings
    if ((left_ir < LF_LINE_THRESH) && (right_ir < LF_LINE_THRESH))
    {
      // Both sensors on black line: move straight ahead
      lspeed = LF_fwSpeed;
      rspeed = LF_fwSpeed;
    } 
    else if (left_ir < LF_LINE_THRESH) 
    {
      // Only left sensor on black: turn left
      lspeed = LF_fwSpeed - LF_ROT_DIFF_SPEED;
      rspeed = LF_fwSpeed + LF_ROT_DIFF_SPEED;
      last_dir = LF_LEFT;
    } 
    else if (right_ir < LF_LINE_THRESH) 
    {
      // Only right sensor on black: turn right
      lspeed = LF_fwSpeed + LF_ROT_DIFF_SPEED;
      rspeed = LF_fwSpeed - LF_ROT_DIFF_SPEED;
      last_dir = LF_RIGHT;
    } 
    else
    {
      // No sensors on black: rotate to find line again
      // Uses last_dir to continue searching in last known direction
      if (last_dir == LF_LEFT)
      {
        lspeed = -LF_ROT_SPEED_AROUND;
        rspeed = LF_ROT_SPEED_AROUND;
      } 
      else
      {
        lspeed = LF_ROT_SPEED_AROUND;
        rspeed = -LF_ROT_SPEED_AROUND;
      }
    }

    // Apply calculated speeds to motors
    kh4_set_speed(lspeed, rspeed, dsPic);

    usleep(20000);  // 20ms delay to prevent CPU overload
  }

  // Clean up when loop exits
  kh4_set_speed(0,0,dsPic);         // Stop robot
  kh4_SetMode(kh4RegIdle,dsPic);    // Set motors to idle
}

/*******************************************************************************
 * Main program entry point
 * Initializes robot communication and starts line following:
 * - Sets up libkhepera and establishes robot connection
 * - Configures interrupt handler
 * - Runs line following algorithm
 * - Performs cleanup on exit
 ******************************************************************************/
int main(int argc, char * argv[])
{
  // Initialize libkhepera
  if (kh4_init(argc ,argv) != 0)
  {
    printf("\nERROR: could not initiate the libkhepera!\n\n");
    return -1;
  }

  // Open communication with robot's dsPic microcontroller
  dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
  if (dsPic == NULL)
  {
    printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
    return -2;
  }

  // Set up interrupt handler for Ctrl+C
  signal(SIGINT, ctrlc_handler);

  // Start line following
  line_following();

  // Clean up before exit
  kh4_set_speed(0, 0, dsPic);                           // Stop robot
  kh4_SetMode(kh4RegIdle, dsPic);                      // Set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic);            // Turn off RGB LEDs

  return 0;
}