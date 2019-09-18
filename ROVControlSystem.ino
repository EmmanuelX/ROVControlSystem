#include <SoftwareSerial.h>
#include <RS485_protocol.h>
#include <Wire.h>
#include "MS5837.h"
#include <Servo.h>

MS5837 sensor;

////////////////////// Define commandArrayByte Elements //////////////////////

#define JSYValFinal_e 0
#define JSXValFinal_e 1
#define JSTValFinal_e 2
#define SWSValFinal_e 3
#define JSBValFinal_e 4
#define SW1ValFinal_e 5
#define SW2ValFinal_e 6
#define SW3ValFinal_e 7
#define SW4ValFinal_e 8
#define SW5ValFinal_e 9
#define SW6ValFinal_e 10
#define POT1ValFinal_e 11
#define POT2ValFinal_e 12
#define POT3ValFinal_e 13

#define commandArrByte_Size 14
byte commandArrByte[commandArrByte_Size];

////////////////////// Define statusBufferArray Elements //////////////////////

#define PressureValFinal_e 0
#define TempValFinal_e 1
#define DepthValFinal_e 2
#define AltitudeValFinal_e 3

#define statusBufferArrByte_Size 4
byte statusBufferArrByte[statusBufferArrByte_Size];

////////////////////// Define statusBufferArray Elements //////////////////////

#define THRHLValFinal_e 0
#define THRHRValFinal_e 1
#define THRVLValFinal_e 2
#define THRVRValFinal_e 3

#define thrusterServo_Size 4
short thrusterServo[thrusterServo_Size];


static const byte THRUSTER_HLEFT = 4;
static const byte THRUSTER_HRIGHT = 5;
static const byte THRUSTER_VLEFT = 6;
static const byte THRUSTER_VRIGHT = 7;


short ThrHL = 0;        // Set thruster values(H=horizontal, V=Vertical, L=Left, R=Right)
short ThrHR = 0;
short ThrVL = 0;
short ThrVR = 0;

short THRHLValFinal;    // output value for the Port Horizontal
short THRHRValFinal;    // output value for the Stbd Horizontal
short THRVLValFinal;    // output value for the Port Vertical
short THRVRValFinal;    // output value for the Port Vertical

short lowThrVal = 1100; // Thrusters take values from 1100 to 1900
short highThrVal = 1900;//1100 = reverse, 1900 = forwards, 1500 = neutral
float thrPowCalc = 0;   //100 = full power with values above, 0 means off

//unsigned long interval = 1000; // the time we need to wait
//unsigned long previousMillis = 0; // millis() returns an unsigned long.

//            RS485 Protocol functions

const byte ENABLE_PIN = 2;
SoftwareSerial rs485 (16, 17);  // receive pin, transmit pin
void fWrite (const byte what){
  rs485.write (what);  
}
int fAvailable (){
  return rs485.available ();  
}
int fRead (){
  return rs485.read ();  
}


Servo servoHorizontalLeft;
Servo servoHorizontalRight;
Servo servoVerticalLeft;
Servo servoVerticalRight;


void setup() {
  
//Communications Setup
  rs485.begin (28800);
  pinMode (ENABLE_PIN, OUTPUT);
  Wire.begin();
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

// Set up Arduino pins to send servo signals to ESCs
  thrusterLeft.attach(THRUSTER_HLEFT);
  thrusterRight.attach(THRUSTER_HRIGHT);
  thrusterVertical.attach(THRUSTER_VLEFT);
  thrusterRight.attach(THRUSTER_VRIGHT);

// Set output signal to 1500 microsecond pulse (stopped command)
  thrusterLeft.writeMicroseconds(1500);
  thrusterRight.writeMicroseconds(1500);
  thrusterVertical.writeMicroseconds(1500);
  thrusterRight.writeMicroseconds(1500);

  // Delay to allow time for ESCs to initialize
  delay(7000); 

}

void loop() {
///////////////////////////////////////////////////////////////////////////////////
//                               Create Status                                   //
///////////////////////////////////////////////////////////////////////////////////

  sensor.read();

  statusBufferArrByte[PressureValFinal_e] = sensor.pressure();  // pressure + mbar
  statusBufferArrByte[TempValFinal_e] = sensor.temperature();   // temperature + deg C
  statusBufferArrByte[DepthValFinal_e] = sensor.depth();        // depth + m
  statusBufferArrByte[AltitudeValFinal_e] = sensor.altitude();  // altitude + m above sea level
  
///////////////////////////////////////////////////////////////////////////////////
//               Send Status to Surface/Wait for Buffer Sent                     //
///////////////////////////////////////////////////////////////////////////////////

  digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  sendMsg (fWrite, statusBufferArrByte, sizeof statusBufferArrByte);
  waitForBuffer();
  digitalWrite (ENABLE_PIN, LOW);  // disable sending 

///////////////////////////////////////////////////////////////////////////////////
//                             Wait for Commands                                 //
///////////////////////////////////////////////////////////////////////////////////

  delay(1);
  byte received = recvMsg (fAvailable, fRead, commandArrByte, sizeof commandArrByte);
  
  if (received){
    setThrusters()  
    }else
    if(!received){
      thrusterServo[THRHLValFinal_e] = 1500;  // Make Sure Values Stay Within thruster's values, and save Final values
      thrusterServo[THRHRValFinal_e] = 1500;
      thrusterServo[THRVLValFinal_e] = 1500;
      thrusterServo[THRVRValFinal_e] = 1500; 
    }

///////////////////////////////////////////////////////////////////////////////////
//                       Send Commands to Thrusters                              //
///////////////////////////////////////////////////////////////////////////////////

  thrusterLeft.writeMicroseconds([THRHLValFinal_e]);
  thrusterRight.writeMicroseconds([THRHRValFinal_e]);
  thrusterVertical.writeMicroseconds(THRVLValFinal_e);
  thrusterRight.writeMicroseconds(THRVRValFinal_e);

/////////////////////////////////////////////////////////////////////////////////////
//                          Send Commands to Tasks                                 //
/////////////////////////////////////////////////////////////////////////////////////




}
void setThrusters(){
   thrPowCalc = ((float)commandArrByte[POT2ValFinal_e] / 100) * 400;
  highThrVal = 1500 + (short)thrPowCalc;
  lowThrVal = 1500 - (short)thrPowCalc;
  
    ThrHL = (128 - commandArrByte[JSYValFinal_e]) - (128 - commandArrByte[JSXValFinal_e]);  // Equations For Joystick Y Values To Set Up Movements For Forward
    ThrHR = (128 - commandArrByte[JSYValFinal_e]) + (128 - commandArrByte[JSXValFinal_e]);  // Backward and YAW Movement. Sets Up Left and Right Horizontal Thrusters
    ThrHL = map(ThrHL, 128, -127, lowThrVal, highThrVal);
    ThrHR = map(ThrHR, 128, -127, lowThrVal, highThrVal);
    if(ThrHL < 0 && ThrHR < 0){
      THRHLValFinal = ThrHR;
      THRHRValFinal = ThrHL;
    }else{
      THRHLValFinal = ThrHL;
      THRHRValFinal = ThrHR;
    }

    if (commandArrByte[SWSValFinal_e] != 0) {                                      // Check If Joystick slider Is Being Used So Vertical Thrusters' Crabbing & Up/Down Movement Do Not Intefere
      THRVLValFinal = map(commandArrByte[SWSValFinal_e], 0, 255, lowThrVal, highThrVal);// If yes, Use Slider Values To Move Up And Down
      THRVRValFinal = map(commandArrByte[SWSValFinal_e], 0, 255, lowThrVal, highThrVal);
    }else {
      THRVLValFinal = map(commandArrByte[JSTValFinal_e], 0, 255, lowThrVal, highThrVal);// If No, Use Twister Values To Crab left Or right
      THRVRValFinal = map(commandArrByte[JSTValFinal_e], 0, 255, lowThrVal, highThrVal);
    }
/*        ---
(ThrHL)--|   |--(ThrHR)
          ---
     (ThrVL)(ThrVR)
*/
  
    thrusterServo[THRHLValFinal_e] = constrain(THRHLValFinal, lowThrVal, highThrVal);  // Make Sure Values Stay Within thruster's values, and save Final values
    thrusterServo[THRHRValFinal_e] = constrain(THRHRValFinal, lowThrVal, highThrVal);
    thrusterServo[THRVLValFinal_e] = constrain(THRVLValFinal, lowThrVal, highThrVal);
    thrusterServo[THRVRValFinal_e] = constrain(THRVRValFinal, lowThrVal, highThrVal); 
}
void waitForBuffer(){
  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
    UCSR0A |= 1 << TXC0;  // mark transmission not complete
  while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
}
