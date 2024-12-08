#include "auton.h"
#include "drive.h"

vex::timer aTimer;

const double turningkP = 0.12;
const double turningkI = 0.0;
const double turningkD = 0;

double turningError = 0.0;
double turningIntegral = 0.0;
double prevTurningError = 0.0;

double desiredTheta = 0.0;

void turnTo(double t){
  aTimer.reset();
  desiredTheta = t;

  while(!(Drivetrain.heading() < desiredTheta + 2 && Drivetrain.heading() > desiredTheta - 2)){
    double theta = Drivetrain.heading();

    turningError = desiredTheta - theta;

    double turningISum = turningIntegral + (turningError * aTimer.time(sec));

    double turningDerivative = (turningError - prevTurningError) / aTimer.time(sec);

    double turningOutput = (turningError * turningkP) + (turningISum * turningkI) + (turningDerivative * turningkD);

    LeftDriveSmart.spin(fwd, turningOutput, volt);
    RightDriveSmart.spin(reverse, turningOutput, volt);

    prevTurningError = turningError;
  }
  Drivetrain.stop();
}

void threeRingGoalRush(){
  Drivetrain.setDriveVelocity(80, pct);

  Drivetrain.driveFor(fwd, 120, inches);
  turnTo(45);
  doinkerOn();
  Drivetrain.driveFor(fwd, 5, inches);
  wait(0.15, sec);
  Drivetrain.setDriveVelocity(50, pct);
  Drivetrain.driveFor(reverse, 15, inches);
}
