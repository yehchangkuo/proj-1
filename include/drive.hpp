#include "main.h"

extern int8_t LFMotor;
extern int8_t LBMotor ;
extern int8_t RFMotor ;
extern int8_t RBMotor ;

extern QLength WheelDiameter;
extern QLength WheelBase ;
extern QLength TrackingWDiameter ;
extern QLength TrackingWBase ;
extern QLength MiddleWheel ;

extern double MAXVELOCITY ;
extern double MAXVOLTAGE ;

void DefaultChassis ();
void EncoderChassisPID ();
