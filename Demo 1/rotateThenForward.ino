#include <Encoder.h>

#define PI 3.1415926535897932384626433832795
#define voltsToPWM 255/5
#define countsToRads (2*PI)/3200;
#define wheelRadius 0.0762        // Units are meters (This equals 3 in.)
#define wheelSeparation 0.254     // Units are meters (This equals 10 in.)
#define DIST -1.0                  // Target distance in feet
#define ANGLE PI                  // Target angle in radians

int enablePin = 4;                // This must be high to run the motor
int voltageSignPinLeft = 7;       // HIGH = Forward, LOW = Backward
int voltageSignPinRight = 8;      // HIGH = Backward, LOW = Forward
int motorVoltagePinLeft = 9;
int motorVoltagePinRight = 10;
double voltageMaximum = 5;

unsigned long currentTime;
double sampleRateInMillis = 10;
double Ts = sampleRateInMillis/1000;

long int currentPosLeftCounts;         // Current position of the right encoder in counts
long int currentPosRightCounts;        // Current position of the left encoder in counts
long double currentPosLeftRads;        // Current position of the right encoder in radians
long double currentPosRightRads;       // Current position of the left encoder in radians
long int newPosLeftCounts;             // Etc.
long int newPosRightCounts;
long double newPosLeftRads;
long double newPosRightRads;

double rhoActual;                       // Instantaneous forward velocity (m/s)
double phiActual;                       // Instantaneous rotational velocity (rad/s)
double rhoDesired;                      // Desired forward velocity
double phiDesired;                      // etc.
double posDesired;
double posError;
double posErrorPast;
double posActual;
double angleDesired;
double angleError;
double angleErrorPast;
double angleActual;
double rhoError;
double phiError;

// Inner loop PI gains
double KpBase = 5;
double KpDelta = 3;
double KiBase = 0.1;
double KiDelta = 0.01;

// Outer loop PD gains
double Kp_Pos = 2;
double Kd_Pos = 0.1;
double Kp_Angle = 3;
double Kd_Angle = 0.1;

// Values used in PID calculations
double I_rho = 0;
double I_phi = 0;
double e_past = 0;
double D_pos;
double D_Angle;

// Velocities obtained from encoder readings
double velocityLeftRadsPerSecond;
double velocityRightRadsPerSecond;

// The inner-loop PI calculates these values, which are sent to the motors
double voltageBase;
double voltageDelta;
double voltageCommandLeft;
double voltageCommandRight;

// FSM states, and flags used to determine when one state is "finished"
#define TURNING 0
#define GOFORWARD 1
#define FINISHED 2
int state = TURNING;
int angleErrorFlag = 0;
int posErrorFlag = 0;

Encoder motorEncoderLeft(2, 6);
Encoder motorEncoderRight(3, 5);

void setup() {
  Serial.begin(250000);
  pinMode(motorVoltagePinRight, OUTPUT);
  pinMode(motorVoltagePinLeft, OUTPUT);
  pinMode(voltageSignPinRight, OUTPUT);
  pinMode(voltageSignPinLeft, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
}

void loop() {
  //rhoDesired = 0.5;
  currentTime = millis();

  // get motor positions, and convert to radians
  currentPosLeftCounts = newPosLeftCounts;       
  currentPosRightCounts = newPosRightCounts;
  currentPosLeftRads = currentPosLeftCounts*countsToRads;
  currentPosRightRads = currentPosRightCounts*countsToRads;

  // We only want our integral terms to count when error is low, to prevent windup
  if (abs(rhoError) < 0.25) {
    I_rho = I_rho + rhoError*sampleRateInMillis;
  } else {
    I_rho = 0;
  }
  if (abs(phiError) < 0.5) {
    I_phi = I_phi + phiError*sampleRateInMillis;
  } else {
    I_phi = 0;
  }

  // Errors used for feedback control
  posError = posDesired - posActual;
  angleError = angleDesired - angleActual;

  // Derivative terms for the outer-loop controller
  if (Ts > 0) {
    D_pos = (posError - posErrorPast)/Ts;
    posErrorPast = posError;
  } else {
    D_pos = 0;
  }
  if (Ts > 0) {
    D_Angle = (angleError - angleErrorPast)/Ts;
    angleErrorPast = angleError;
  } else {
    D_Angle = 0;
  }

  phiActual = -wheelRadius*((velocityLeftRadsPerSecond - velocityRightRadsPerSecond)/wheelSeparation);
  rhoActual = wheelRadius*((velocityLeftRadsPerSecond + velocityRightRadsPerSecond)/2);
  rhoError = rhoActual - rhoDesired;
  phiError = phiActual - phiDesired;

  // Calculating motor voltages
  voltageBase = (KpBase*rhoError)+(KiBase*I_rho);
  voltageDelta = (KpDelta*phiError)+(KiDelta*I_phi);
  voltageCommandLeft = (voltageBase-voltageDelta)/2;
  voltageCommandRight = (voltageBase+voltageDelta)/2;

  // Determining motor direction - if our voltage is negative, make it positive and reverse the motor
  if (voltageCommandLeft < 0) {
    digitalWrite(voltageSignPinLeft, LOW);
    voltageCommandLeft *= -1;
  } else {
    digitalWrite(voltageSignPinLeft, HIGH);
  }
  if (voltageCommandRight < 0) {
    digitalWrite(voltageSignPinRight, HIGH);
    voltageCommandRight *= -1;
  } else {
    digitalWrite(voltageSignPinRight, LOW);
  }
  
  // Manually saturating the voltage to prevent slipping
  if (voltageCommandLeft > voltageMaximum) {
    voltageCommandLeft = voltageMaximum;
  }
  if (voltageCommandRight > voltageMaximum) {
    voltageCommandRight = voltageMaximum;
  }

  // Write a PWM signal to the motors using the voltageCommands
  analogWrite(motorVoltagePinLeft, voltageCommandLeft*voltsToPWM);
  analogWrite(motorVoltagePinRight, voltageCommandRight*voltsToPWM);

  // Getting new positions from the motor encoders
  newPosLeftCounts = -motorEncoderLeft.read();           // Negative because we want the encoder to tick up when this wheel moves forward
  newPosRightCounts = motorEncoderRight.read();
  newPosLeftRads = newPosLeftCounts*countsToRads;
  newPosRightRads = newPosRightCounts*countsToRads;

  // Use positions to calculate velocity
  velocityLeftRadsPerSecond = ((newPosLeftRads-currentPosLeftRads)/(sampleRateInMillis))*1000;
  velocityRightRadsPerSecond = ((newPosRightRads-currentPosRightRads)/(sampleRateInMillis))*1000;

  // Update our position and angle with rho and phi data
  posActual += (rhoActual*(sampleRateInMillis))/1000;
  angleActual += (phiActual*(sampleRateInMillis))/1000;



  if (state == TURNING) {
    // To prevent slipping, we have the target angle as a ramp input, not a step input
    if(abs(angleDesired) < abs(ANGLE)) {
      angleDesired += ANGLE/100.0;
    }
    
    posDesired = 0; // We don't want any forward motion while turning
    
    voltageMaximum = 2; // Manually saturating voltage, again to prevent slipping

    // Outer loop PD controller to determine phi and rho
    phiDesired = (angleError*Kp_Angle)+(D_Angle*Kd_Angle);
    rhoDesired = (posError*Kp_Pos)+(D_pos*Kd_Pos);

    // Once we spend enough time with low error, we move to the next state
    if (angleError < 0.01) {
      angleErrorFlag += 1;
    } else {
      angleErrorFlag = 0;
    }
    if (angleErrorFlag >= 350) {
      state = GOFORWARD;
    }
    
  } else if (state == GOFORWARD) {

    // Desired position ramp input - DIST is multiplied by 0.3048 to convert from feet to meters
    if(abs(posDesired) < abs(0.3048*DIST)) {
      posDesired += 0.3048*DIST/100.0;
    }

    // No need to saturate voltage, so we set it back to 5
    voltageMaximum = 5;

    // Outer loop PD controller to determine phi and rho
    phiDesired = (angleError*Kp_Angle)+(D_Angle*Kd_Angle);
    rhoDesired = (posError*Kp_Pos)+(D_pos*Kd_Pos);


    // Once we spend enough time with low error, we move to the next state
    if (posError < 0.01) {
      posErrorFlag += 1;
    } else {
      posErrorFlag = 0;
    }
    if (posErrorFlag >= 350) {
      state = FINISHED;
    }
    
  } else if (state == FINISHED) {
    // When finished, we want to stop all motion
    phiDesired = 0;
    rhoDesired = 0;
  }

  // Print a bunch of data for debugging purposes
  Serial.print(posDesired);
  Serial.print("  ");
  Serial.print(posActual);
  Serial.print("  ");
  Serial.print(rhoDesired);
  Serial.print("  ");
  Serial.print(rhoActual);
  Serial.print("  ");
  Serial.println(state);
  
  while(millis() < currentTime + sampleRateInMillis) {
    // Wait until the sample time has passed
  }

}
