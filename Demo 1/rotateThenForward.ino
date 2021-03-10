#include <Encoder.h>

#define PI 3.1415926535897932384626433832795
#define voltsToPWM 255/5
#define countsToRads (2*PI)/3200;
#define wheelRadius 0.0762        // Units are meters (This equals 3 in.)
#define wheelSeparation 0.254     // Units are meters (This equals 10 in.)

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
double rhoDesired;
double phiDesired;
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

double KpBase = 5;
double KpDelta = 3;
double KiBase = 0.1;
double KiDelta = 0.001;

double Kp_Pos = 3;
double Kd_Pos = 0.1;
double Kp_Angle = 10;
double Kd_Angle = 0.001;

double I_rho = 0;
double I_phi = 0;
double e_past = 0;
double D_pos;
double D_Angle;

double velocityLeftRadsPerSecond;
double velocityRightRadsPerSecond;

double voltageBase;
double voltageDelta;
double voltageCommandLeft;
double voltageCommandRight;

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


  //I_rho = I_rho + rhoError*sampleRateInMillis;

  if (abs(rhoError) < 0.25) {
    I_rho = I_rho + rhoError*sampleRateInMillis;
  } else {
    I_rho = 0;
  }

  //if (abs(phiError) < 0.5) {
    I_phi = I_phi + phiError*sampleRateInMillis;
  //} else {
  //  I_phi = 0;
  //}
  
  posError = posDesired - posActual;
  angleError = angleDesired - angleActual;

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

  voltageBase = (KpBase*rhoError)+(KiBase*I_rho);
  voltageDelta = (KpDelta*phiError)+(KiDelta*I_phi);

  phiActual = -wheelRadius*((velocityLeftRadsPerSecond - velocityRightRadsPerSecond)/wheelSeparation);
  rhoActual = wheelRadius*((velocityLeftRadsPerSecond + velocityRightRadsPerSecond)/2);

  rhoError = rhoActual - rhoDesired;
  phiError = phiActual - phiDesired;

  voltageCommandLeft = (voltageBase-voltageDelta)/2;
  voltageCommandRight = (voltageBase+voltageDelta)/2;

  if (voltageCommandLeft < 0) {
    digitalWrite(voltageSignPinLeft, HIGH); //negative voltage = go backward
    voltageCommandLeft *= -1;
  } else {
    digitalWrite(voltageSignPinLeft, LOW);
  }

  if (voltageCommandRight < 0) {
    digitalWrite(voltageSignPinRight, LOW);
    voltageCommandRight *= -1;
  } else {
    digitalWrite(voltageSignPinRight, HIGH);
  }

  if (voltageCommandLeft > voltageMaximum) {
    voltageCommandLeft = voltageMaximum;
  }
  if (voltageCommandRight > voltageMaximum) {
    voltageCommandRight = voltageMaximum;
  }

  // Write to the motors
  analogWrite(motorVoltagePinLeft, voltageCommandLeft*voltsToPWM);
  analogWrite(motorVoltagePinRight, voltageCommandRight*voltsToPWM);
  
  newPosLeftCounts = -motorEncoderLeft.read();           // Negative because we want the encoder to tick up when this wheel moves forward
  newPosRightCounts = motorEncoderRight.read();            // Modulo 3200 because 3200 counts = 1 rotation
  newPosLeftRads = newPosLeftCounts*countsToRads;
  newPosRightRads = newPosRightCounts*countsToRads;

  velocityLeftRadsPerSecond = ((newPosLeftRads-currentPosLeftRads)/(sampleRateInMillis))*1000;
  velocityRightRadsPerSecond = ((newPosRightRads-currentPosRightRads)/(sampleRateInMillis))*1000;

  posActual += (rhoActual*(sampleRateInMillis))/1000;
  angleActual += (phiActual*(sampleRateInMillis))/1000;

  if (state == TURNING) {

    if (angleDesired < PI/2) {
      angleDesired += PI/2000;
    }
    
    voltageMaximum = 5;
    
    phiDesired = (angleError*Kp_Angle)+(D_Angle*Kd_Angle);
    rhoDesired = 0;
    
    if (angleError < 0.01) {
      angleErrorFlag += 1;
    } else {
      angleErrorFlag = 0;
    }

    if (angleErrorFlag >= 500) {
      state = GOFORWARD;
    }
    
  } else if (state == GOFORWARD) {

    voltageMaximum = 5;
    phiDesired = (angleError*Kp_Angle)+(D_Angle*Kd_Angle);
    rhoDesired = (posError*Kp_Pos)+(D_pos*Kd_Pos);

    if (posError < 0.01) {
      posErrorFlag += 1;
    } else {
      posErrorFlag = 0;
    }

    if (posErrorFlag >= 500) {
      state = FINISHED;
    }
    
  } else if (state == FINISHED) {
    phiDesired = 0;
    rhoDesired = 0;
  }

  Serial.print(angleDesired);
  Serial.print("  ");
  Serial.print(angleActual);
  Serial.print("  ");
  Serial.print(phiDesired);
  Serial.print("  ");
  Serial.print(phiActual);
  Serial.print("  ");
  Serial.println(state);
  
  while(millis() < currentTime + sampleRateInMillis) {
    // Wait until the sample time has passed
  }

}
