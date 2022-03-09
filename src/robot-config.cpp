#include "vex.h"
#include "math.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;
brain Brain;

// VEXcode device constructors
//bumper frontBumper = bumper(triport::port &port)

motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT10, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT11, ratio18_1, true);
motor rightMotorB = motor(PORT20, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 165, mm, 1);
controller Controller1 = controller(primary);

// some vars
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;


// exponential speed control left side
int getLeftDriveSpeed (vex::directionType type, int percentage)
{
  if (percentage >= 0)
  {
    // when stationary, or going forwards, put the percentage in the exponential func
    percentage = 1.2 * pow(1.043, percentage) - 1.2 + 0.2 * percentage;
    return percentage; 
  }
  else
  {
    // when reversing, take the positive percentage, and put it in exponential func
    percentage = -percentage;
    percentage = 1.2 * pow(1.043, percentage) - 1.2 + 0.2 * percentage; 
    
    // when the new percentage val is calculated, make negative for backwards movement
    percentage = -percentage;
    return percentage;
  }
}

// exponential speed control right side
int getRightDriveSpeed (vex::directionType type, int percentage)
{
  if (percentage >= 0)
  {
    // when stationary, or going forwards, put the percentage in the exponential func
    percentage = 1.2 * pow(1.043, percentage) - 1.2 + 0.2 * percentage;
    return percentage; 
  }
  else
  {
    // when reversing, take the positive percentage, and put it in exponential func
    percentage = -percentage;
    percentage = 1.2 * pow(1.043, percentage) - 1.2 + 0.2 * percentage; 
    
    // when the new percentage val is calculated, make negative for backwards movement
    percentage = -percentage;
    return percentage;
  }
}


// function that adjusts speed based on what the buttons do
int getTagMultiplier (int tagMultiplier)
{
  
  return tagMultiplier;
}



int rc_auto_loop_function_Controller1() 
{
  while(true) 
  {
    int deadband = 2;

    // still need to add speed multiplier
    int leftMotorSpeed = getLeftDriveSpeed (vex::directionType::fwd, Controller1.Axis3.value());
    int rightMotorSpeed = getRightDriveSpeed (vex::directionType::fwd, Controller1.Axis2.value());
    
    // left side deadband checking
    if (abs(leftMotorSpeed) < deadband)
    {
      LeftDriveSmart.setVelocity(0, percent);
    }
    else
    {
      LeftDriveSmart.setVelocity(leftMotorSpeed, percent);
    }

    // right side deadband checking
    if (abs(rightMotorSpeed) < deadband)
    {
      RightDriveSmart.setVelocity(0, percent);
    }
    else
    {
      RightDriveSmart.setVelocity(rightMotorSpeed, percent);
    }
    
    // spin both motors
    RightDriveSmart.spin(forward);
    LeftDriveSmart.spin(forward);

    wait(20, msec);
  }

  return 0;
}



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}