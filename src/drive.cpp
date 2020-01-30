#include "main.h"
#include "drive.hpp"

int8_t LFMotor= 1;
int8_t LBMotor = 3;
int8_t RFMotor = -2;
int8_t RBMotor = -12;

QLength WheelDiameter = 4_in;
QLength WheelBase = 11.5_in;             // 11.5_in;
QLength TrackingWDiameter = 2.75_in;
QLength TrackingWBase = 9_in;       // 7_in
QLength XTrackingWDiameter = 2.75_in;        // 1_in
QLength XTrackingWBase = 3.5_in;

double MAXVELOCITY = 100;
double MAXVOLTAGE = 10000;


void DefaultChassis () {
    auto chassis = ChassisControllerBuilder()
    .withMotors(
      {LFMotor,LBMotor},
      {RFMotor, RBMotor}) // Left motor is 1 , 3, right motor is 2, 4 (reversed)
    // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{WheelDiameter, WheelBase}, imev5GreenTPR})
    // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
    .withOdometry()
    .buildOdometry();

    chassis->setState({0_in, 0_in, 0_deg});
    // turn 45 degrees and drive approximately 1.4 ft
    chassis->driveToPoint({2_ft, 2_ft});
    // turn approximately 45 degrees to end up at 90 degrees
    chassis->turnToAngle(90_deg);
    // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
    chassis->turnToPoint({1_ft, 0_ft});
  }

void EncoderChassisPID () {
  auto chassis =  ChassisControllerBuilder()
  .withMotors(
    {LFMotor,LBMotor},
    {RFMotor, RBMotor}) // Left motor is 1 , 3, right motor is 2, 4 (reversed)
  // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withSensors(
        ADIEncoder{'E', 'F'}, // Left encoder in ADI ports A & B
        ADIEncoder{'G', 'H', true},  // Right encoder in ADI ports C & D (reversed)
        ADIEncoder{'A', 'B'}  // middle encoder in ADI ports E & F
     )
    .withGains(
      {0.001, 0, 0.0001}, // Distance controller gains
      {0.001, 0, 0.0001}, // Turn controller gains
      {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    .withDimensions(AbstractMotor::gearset::green, {{TrackingWDiameter, TrackingWBase, XTrackingWDiameter, XTrackingWBase}, imev5GreenTPR})
    .withDerivativeFilters(
      std::make_unique<AverageFilter<3>>(), // Distance controller filter
      std::make_unique<AverageFilter<3>>(), // Turn controller filter
      std::make_unique<AverageFilter<3>>()  // Angle controller filter
    )
    // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
    .withOdometry({{TrackingWDiameter, TrackingWBase}, quadEncoderTPR})
    .withMaxVoltage(MAXVOLTAGE)
    .withMaxVelocity(MAXVELOCITY)
    .buildOdometry() ;
    chassis->setState({0_in, 0_in, 0_deg});
    // turn 45 degrees and drive approximately 1.4 ft
    chassis->driveToPoint({2_ft, 2_ft});
    // turn approximately 45 degrees to end up at 90 degrees
    chassis->turnToAngle(90_deg);
    // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
    chassis->turnToPoint({1_ft, 0_ft});

}
