#include <SoftwareSerial.h>
#include <RS485_protocol.h>

#include <LiquidCrystal.h>      //Library for 16x02 LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

////////////////////// Define commandArrayByte Elements //////////////////////

#define JSYValFinal_e 0     // Final control values sent to ROV
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
byte commandArrByte[commandArrByte_Size];     // Array holds pack of data sent to ROV

////////////////////// Define statusBufferArray Elements //////////////////////

#define PressureValFinal_e 0
#define TempValFinal_e 1
#define DepthValFinal_e 2
#define AltitudeValFinal_e 3

#define statusBufferArrByte_Size 4
byte statusBufferArrByte[statusBufferArrByte_Size];

const byte pot1Pin = 5;          // Potentiometer Pins
const byte pot2Pin = 6;
const byte pot3Pin = 7;

const byte jsxPin = 8;           // Joystick pins: X-axis
const byte jsyPin = 9;           // Joystick pins: Y-axis
const byte jstPin = 10;          // Joystick pins: Twister 
const byte jsbPin = 37;          // Joystick pins: Button Pin
const byte swsPin = 11;          // Slider Switch Pin

const byte sw1Pin = 22;          // Switches' Pins
const byte sw2Pin = 23;
const byte sw3Pin = 24;
const byte sw4Pin = 25;
const byte sw5Pin = 26;
const byte sw6Pin = 27;

const byte led1G = 42;           //Pins To Control LEDs: ex.analogWrite(led1G, 255);
const byte led1R = 43;
const byte led2G = 44;
const byte led2R = 45;
const byte led3G = 46;
const byte led3R = 47;
const byte led4G = 48;
const byte led4R = 49;

short pot1Val = 0;        // Get Values for Potentiometer
short pot2Val = 0;
short pot3Val = 0;

short POT1ValFinal = 0;   // Get Final Values for Potentiometer
short POT2ValFinal = 0;
short POT3ValFinal = 0;

short jsxVal = 0;         // Get Values for Joystick, Slider Pins
short jsyVal = 0;
short jstVal = 0;
short swsVal = 0;

byte JSXValFinal = 0;     // Get Final Values for Joystick, Slider Pins
byte JSYValFinal = 0;
byte JSTValFinal = 0;
byte SWSValFinal = 0;
byte JSBValFinal = 1;

byte SW1ValFinal = 0;
byte SW2ValFinal = 0;
byte SW3ValFinal = 0;
byte SW4ValFinal = 0;
byte SW5ValFinal = 0;
byte SW6ValFinal = 0;

int deadLow = 485;        // Keeping Joystick & Slider Values Between
int deadHigh = 525;       // 485 - 525 Will Not Move ROV

/////////////////RS485 Protocol functions////////////////////

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


void setup() {

  pinMode(led1G, OUTPUT);
  pinMode(led1R, OUTPUT);
  pinMode(led2G, OUTPUT);
  pinMode(led2R, OUTPUT);
  pinMode(led3G, OUTPUT);
  pinMode(led3R, OUTPUT);
  pinMode(led4G, OUTPUT);
  pinMode(led4R, OUTPUT);
  pinMode(jsbPin, INPUT);  

//Setup LCD
  lcd.begin(16, 2);               // Start The Library
  lcd.clear();                    // Clear previous LCD Screen and set Cursor to 0
  lcd.print("Welcome"); 
  delay(1500);
  
  rgbLED(led1R, 1, led1G, 0);     // Set All LED's Red
  rgbLED(led2R, 1, led2G, 0);
  rgbLED(led3R, 1, led3G, 0);
  rgbLED(led4R, 1, led4G, 0);

//Communications Setup
  rs485.begin (28800);
  pinMode (ENABLE_PIN, OUTPUT);
  Serial.setTimeout(100);
}

void loop() {

///////////////////////////////////////////////////////////////////////////////////
//             Read Values from Control Box                                      //
///////////////////////////////////////////////////////////////////////////////////

  jsyVal = analogRead(jsyPin);
  jsxVal = analogRead(jsxPin);
  jstVal = analogRead(jstPin);
  swsVal = analogRead(swsPin);
  JSBValFinal = digitalRead(jsbPin);
  pot1Val = analogRead(pot1Pin);
  pot2Val = analogRead(pot2Pin);
  pot3Val = analogRead(pot3Pin);

  SW1ValFinal = digitalRead(sw1Pin);
  SW2ValFinal = digitalRead(sw2Pin);
  SW3ValFinal = digitalRead(sw3Pin);
  SW4ValFinal = digitalRead(sw4Pin);
  SW5ValFinal = digitalRead(sw5Pin);
  SW6ValFinal = digitalRead(sw6Pin);

  
///////////////////////////////////////////////////////////////////////////////////
//                     Map Values to Send Down                                   //
///////////////////////////////////////////////////////////////////////////////////
 
  jsyVal = checkDead(jsyVal, deadHigh, deadLow);  // Make sure small Joystick movements don't affect the ROV
  jsxVal = checkDead(jsxVal, deadHigh, deadLow);
  jstVal = checkDead(jstVal, deadHigh, deadLow);
  swsVal = checkDead(swsVal, deadHigh, deadLow);
  
  jsyVal = map(jsyVal, 0, 1023, 0, 255);          //0 = reverse. Top = 1023, Left = 1023
  jsxVal = map(jsxVal, 0, 1023, 0, 255);          //Setting 255 first reverses values and makes right Joystick values positive
  jstVal = map(jstVal, 0, 1023, 0, 255);
  swsVal = map(swsVal, 0, 1023, 0, 255);
  
  JSYValFinal = constrain(jsyVal, 0, 255);        // Keep the values between 0 to 255
  JSXValFinal = constrain(jsxVal, 0, 255);
  JSTValFinal = constrain(jstVal, 0, 255);
  SWSValFinal = constrain(swsVal, 0, 255);
/*    +255
  +255  |  0
        0
*/
  POT1ValFinal = map(pot1Val, 0, 1023, 0, 255);   // Map values to 255(Byte-Values)
  POT2ValFinal = map(pot2Val, 0, 1023, 0, 100);
  POT3ValFinal = map(pot3Val, 0, 1023, 0, 100);

  
///////////////////////////////////////////////////////////////////////////////////
//                   (Assemble)Pack Values in Arrays                             //
///////////////////////////////////////////////////////////////////////////////////
  
  commandArrByte[0] = JSYValFinal;
  commandArrByte[1] = JSXValFinal;
  commandArrByte[2] = JSTValFinal;
  commandArrByte[3] = SWSValFinal;
  commandArrByte[4] = JSBValFinal;
  commandArrByte[5] = SW1ValFinal;
  commandArrByte[6] = SW2ValFinal;
  commandArrByte[7] = SW3ValFinal;
  commandArrByte[8] = SW4ValFinal;
  commandArrByte[9] = SW5ValFinal;
  commandArrByte[10] = SW6ValFinal;
  commandArrByte[11] = POT1ValFinal;
  commandArrByte[12] = POT2ValFinal;
  commandArrByte[13] = POT3ValFinal;

///////////////////////////////////////////////////////////////////////////////////
//                               Wait For Status Array                           //
///////////////////////////////////////////////////////////////////////////////////

  byte received = recvMsg (fAvailable, fRead, statusBufferArrByte, sizeof statusBufferArrByte);
  if(received == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error!");
  }

///////////////////////////////////////////////////////////////////////////////////
//                           Send Command Arrays                                 //
///////////////////////////////////////////////////////////////////////////////////

  digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  sendMsg (fWrite, commandArrByte, sizeof commandArrByte);
  waitForBuffer();
  digitalWrite (ENABLE_PIN, LOW);  // disable sending 

///////////////////////////////////////////////////////////////////////////////////
//                             Display Status                                    //
///////////////////////////////////////////////////////////////////////////////////

  //printThrusterValues2(hL, hR, vL, vR);
}

static int checkDead(int joyS, int high, int low){

  if(joyS < high && joyS > low){
    joyS = 512;
  }
  return joyS;
}

static double mapMtrRot(double x, double in_min, double in_max, double out_min, double out_max){
        
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

void rgbLED(int redLED, int redOn,  int greenLED, int greenOn){
  
  if(redOn == 0){
    analogWrite(redLED, 255);
  }else
  if(redOn == 1){
    analogWrite(redLED, 0);
  }
  if(greenOn == 0){
    analogWrite(greenLED, 255);
  }else
  if(greenOn == 1){
    analogWrite(greenLED, 0);
  }

}

void waitForBuffer(){
  while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
    UCSR0A |= 1 << TXC0;  // mark transmission not complete
  while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
}

void printJoystickFinalVal(short jSyVal, short jSxVal, short jStVal, short swSVal){

  lcd.setCursor(0,0);           
  lcd.print("jY "); 
  lcd.print(jSyVal);
  lcd.setCursor(8,0);
  lcd.print(" jX ");
  lcd.print(jSxVal);
  lcd.setCursor(0,1);
  lcd.print("jT "); 
  lcd.print(jStVal);
  lcd.setCursor(8,1);
  lcd.print(" sS ");
  lcd.print(swSVal); 
  delay(100);
  lcd.clear();
}

void printThrusterValues2(short hL, short hR, short vL, short vR){

  lcd.setCursor(0,0);           
  lcd.print("hL "); 
  lcd.print(hL);
  lcd.setCursor(8,0);
  lcd.print(" hR ");
  lcd.print(hR);
  lcd.setCursor(0,1);
  lcd.print("vL "); 
  lcd.print(vL);
  lcd.setCursor(8,1);
  lcd.print(" vR ");
  lcd.print(vR); 
  delay(100);
  lcd.clear();
}

void printmtrRot(short pot1Val){

  String isCW = "CW";
  double MTRFinal = 0;
  boolean cw = true;
  
  MTRFinal = mapMtrRot(pot1Val, -100, 100, -15, 15);

  if(MTRFinal < 0){
    MTRFinal = MTRFinal * -1;
    isCW = "C-CW";
  }
  
    
  lcd.setCursor(0,0);           
  lcd.print("Motor Rotations"); 
  lcd.setCursor(0,1);  
  lcd.print(MTRFinal);
  lcd.setCursor(6,0);
  lcd.print(isCW);
  
  delay(500);
  lcd.clear();


}
