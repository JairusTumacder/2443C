#include "drive.h"

void runIntake(double speed){
  intake.spin(fwd, speed, pct);
  conveyor.spin(fwd, speed, pct);
}

void runOuttake(double speed){
  intake.spin(reverse, speed, pct);
  conveyor.spin(reverse, speed, pct);
}

void stopIntake() {
  intake.stop();
  conveyor.stop();
}

void mogoOn(){
  mogo.set(true);
}

void mogoOff(){
  mogo.set(false);
}

void doinkerOn(){
  doinker.set(true);
}

void doinkerOff(){
  doinker.set(false);
}

void toggleMogoMech() {
  if (!mogo.value()) {
    mogoOn();
  } else {
    mogoOff();
  }
}

void toggleDoinkerMech() {
  if (!doinker.value()) {
    doinkerOn();
  } else {
    doinkerOff();
  }
}

vex::timer myTimer;

const double kP = 0.25;
const double kI = 0;
const double kD = 0.1;

double error = 0.0;
double integral = 0.0;
double prevError = 0.0;

double desiredArmTheta = 0.0;

bool enableArm = false;

void runArmPD(double t){
  arm.setBrake(brakeType::hold);
  myTimer.reset();
  desiredArmTheta = t;
  enableArm = true;
}

void armPID(){
  while (enableArm) {
    double theta = rot.angle();

    error = desiredArmTheta - theta;

    double derivative = (error - prevError) / myTimer.time(timeUnits::sec);

    double iSum = integral + (error * myTimer.time(timeUnits::sec));

    double output = (kP * error) + (kI * iSum) + (kD * derivative);

    arm.spin(directionType::fwd, output, voltageUnits::volt);

    prevError = error;

    if (rot.angle() > desiredArmTheta - 2 && rot.angle() < desiredArmTheta + 2) {
      enableArm = false;
      break;
    }
  }
  
  arm.stop();
}

void driveControl() {
  Controller1.ButtonL1.pressed(toggleMogoMech);
  Controller1.ButtonX.pressed(toggleDoinkerMech);

  if (Controller1.ButtonR1.pressing()) {
    runIntake(127);
  } else if (Controller1.ButtonL2.pressing()) {
    runIntake(30);
  } else if (Controller1.ButtonR2.pressing()) {
    runOuttake(127);
  } else {
    stopIntake();
  }

  arm.setBrake(brakeType::hold);

  if(Controller1.ButtonUp.pressing()){
    arm.spin(directionType::fwd, 50, percentUnits::pct);
  }
  else if(Controller1.ButtonDown.pressing()){
    arm.spin(directionType::rev, 50, percentUnits::pct);
  }
  else{
    arm.stop();
  }
}