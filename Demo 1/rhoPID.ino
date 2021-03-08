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
double posDesired = 1;
double posError;
double posErrorPast;
double posActual;
double angleDesired = 0;
double angleError;
double angleErrorPast;
double angleActual;
double rhoError;
double phiError;

double KpBase = 5;
double KpDelta = 0.5;
double KiBase = 0.02;
double KiDelta = 0.01;

double Kp_Pos = 2;
double Kd_Pos = 0.0001;
double Kp_Angle = 0.5;
double Kd_Angle = 0.05;

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
  currentTime = millis();

  // get motor positions, and convert to radians
  currentPosLeftCounts = newPosLeftCounts;       
  currentPosRightCounts = newPosRightCounts;
  currentPosLeftRads = currentPosLeftCounts*countsToRads;
  currentPosRightRads = currentPosRightCounts*countsToRads;

  
  

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

  rhoDesired = (posError*Kp_Pos)+(D_pos*Kd_Pos);
  phiDesired = (angleError*Kp_Angle)+(D_Angle*Kd_Angle);

  

  phiActual = -wheelRadius*((velocityLeftRadsPerSecond - velocityRightRadsPerSecond)/wheelSeparation);
  rhoActual = wheelRadius*((velocityLeftRadsPerSecond + velocityRightRadsPerSecond)/2);

  rhoError = rhoActual - rhoDesired;
  phiError = phiActual - phiDesired;

  if (abs(rhoError) < 0.05) {
    I_rho = I_rho + rhoError*sampleRateInMillis;
  } else {
    I_rho = 0;
  }

  if (abs(phiError) < 0.05) {
    I_phi = I_phi + phiError*sampleRateInMillis;
  } else {
    I_phi = 0;
  }

  voltageBase = (KpBase*rhoError)+(KiBase*I_rho);
  voltageDelta = (KpDelta*rhoError)+(KiDelta*I_phi);

  voltageCommandLeft = (voltageBase+voltageDelta)/2;
  voltageCommandRight = (voltageBase-voltageDelta)/2;
  
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

  if (voltageCommandLeft > 5) {
    voltageCommandLeft = 5;
  }
  if (voltageCommandRight > 5) {
    voltageCommandRight = 5;
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

  Serial.print(rhoError);
  Serial.print("     ");
  Serial.print(rhoActual);
  Serial.print("     ");
  Serial.print(rhoDesired);
  Serial.print("     ");
  Serial.print(posActual);
  Serial.print("     ");
  Serial.println(posDesired);
  
  while(millis() < currentTime + sampleRateInMillis) {
    // Wait until the sample time has passed
  }

}
