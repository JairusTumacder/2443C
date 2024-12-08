using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern inertial DrivetrainInertial;
extern motor leftMotorC;
extern motor rightMotorC;
extern motor intake;
extern motor conveyor;
extern motor arm;
extern digital_out mogo;
extern digital_out doinker;
extern rotation rot;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );