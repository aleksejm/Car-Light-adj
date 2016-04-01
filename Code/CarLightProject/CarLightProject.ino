/*----------------------------------------------------------------------*
 *                                                                      *
   An example that uses both short and long button presses.
 *                                                                      *
  A simple state where a short press of the button turns on blink of
  turn led, and a long press turns the side Light LED on.
   
  Driving two steppers concurrently with the Arduino Nano.
  stepper1 follows two potentiometers on analog input 2 and 3.
  stepper2 follows rotary encoder on digital inputm 2, 3, 12.
  A timeout is used to turn off coils and save energy in battery
  operated applications.

  Hardware:
  Arduino NANO.
  Drivers: ULN2003A
  Stepper Motors: 28BYJ48, 5VDC, step angle 5.625 °
               Gear reduction 64:1
               No-load starting frequency:> = 500PPS (4 rpm)
               Coil resistance 60 Ohms.
  -----------------------------------------------------*/
#include <MultiStepper.h>     //https://arduino-info.wikispaces.com/SmallSteppers
#include <AccelStepper.h>     //same as above.

#include <Button.h>           //https://github.com/JChristensen/Button
#include <Stepper.h>          //http://arduining.com/category/driving-motors/

#define  STEPS       4096        // 64(fullsteps) * 64 (reduction ratio)
#define  SPEED       4      // motor speed (RPM)

#define  COIL1       8        //Stepper1 make adjusting of headlight hight
#define  COIL2       9
#define  COIL3       10
#define  COIL4       11

//#define  COIL5     0        //Stepper2 make adjusting of headlight horizontal position
//#define  COIL6     1
//#define  COIL7     2
//#define  COIL8     3

#define  POTFR       A2
#define  POTRR       A3
#define  TIMEOUT     1000     //Turns off after 1 sec of inactivity.
#define  NOISE       8        //inmunity in steps because analog noisy readings.

#define BUTTONL_PIN  A0       //Connect a tactile button switch (or something similar) from Arduino pin 2 to ground.
#define BUTTONR_PIN  A1       //For Right turn switch
#define PULLUP true           //To keep things simple, we use the Arduino's internal pullup resistor.
#define INVERT true           //Since the pullup resistor will keep the pin high unless the
                              //switch is closed, this is negative logic, i.e. a high state
                              //means the button is NOT pressed. (Assuming a normally open switch.)
#define DEBOUNCE_MS   20      //A debounce time of 20 milliseconds usually works well for tactile button switches.

#define LEDL_PIN       7      //The standard Arduino "Pin 2" LED Left.
#define LEDR_PIN       4      //The standard Arduino "Pin 3" LED Right.
#define LEDLF_PIN      5      //The standard Arduino "Pin 5" LED Left Fade.
#define LEDRF_PIN      6      //The standard Arduino "Pin 6" LED Right Fade.
#define LONG_PRESS     1200   //We define a "long press" to be 1000 milliseconds.
#define SHORT_PRESS    300
#define BLINK_INTERVAL 300    //In the BLINK state, switch the LED every 100 milliseconds.

//============== For Rotary start code====================================================
// NOTE: The sequence 1-3-2-4 is required for proper sequencing of 28BYJ48
// create an instance of the stepper class:
//Stepper stepper1(STEPS, COIL1, COIL2, COIL3, COIL4);
AccelStepper stepper(STEPS, COIL1, COIL3, COIL2, COIL4);
//AccelStepper stepper2(STEPS, COIL5, COIL7, COIL6, COIL8);

//============== Code ending for Rotary & Starting for buttons ==========================
Button myBtnL(BUTTONL_PIN, PULLUP, INVERT, DEBOUNCE_MS);    //Declare the button Left
Button myBtnR(BUTTONR_PIN, PULLUP, INVERT, DEBOUNCE_MS);    //Declare the button Right

//The list of possible states for the state machine. This state machine has a fixed
//sequence of states, i.e. ONOFF --> TO_BLINK --> BLINK --> TO_ONOFF --> ONOFF
//Note that while the user perceives two "modes", i.e. ON/OFF mode and rapid blink mode,
//two extra states are needed in the state machine to transition between these modes.
enum {ONOFF, TO_BLINK, BLINK, TO_ONOFF};

uint8_t STATE;                   //The current state machine state
boolean ledStateR, ledStateL;    //The current LED status
unsigned long ms;                //The current time from millis()
unsigned long msLast;            //The last time the LED was switched

int pos = 0 ;                    //Position in steps(0-630)= (0°-315°)
int newpos;
unsigned long stamp;             // last move time stamped.

int brightnessL = 0;             // how bright the LED is
int fadeAmountL = 55;            // how many points to fade the LED by
int brightnessR = 0;             // how bright the LED is
int fadeAmountR = 55;            // how many points to fade the LED by
int myResult = 2;                //times to blink

//-----------------------------------------------------------------------------
//Check for inactivity and turn off the steppers coils to save battery.
void CheckTimeout() {
  if ((millis() - stamp) > TIMEOUT) { //Turn Off StepperX coils.
    digitalWrite(COIL1, LOW);
    digitalWrite(COIL2, LOW);
    digitalWrite(COIL3, LOW);
    digitalWrite(COIL4, LOW);
  }
/*  if ((millis() - stampY) > TIMEOUT) { //Turn Off StepperY coils.
    digitalWrite(COIL5, LOW);
    digitalWrite(COIL6, LOW);
    digitalWrite(COIL7, LOW);
    digitalWrite(COIL8, LOW);
  }*/
}

void setup(void){
  {
  stepper.setMaxSpeed(1000.0);       // set the motor max speed.
  stepper.setAcceleration(100.0);    // set the motor Acceleration.
  stepper.setSpeed(SPEED);           // set the motor speed.
  
//  stepper2.setMaxSpeed(1000.0);      // set the motor max speed.
//  stepper2.setAcceleration(50.0);    // set the motor Acceleration.
//  stepper2.setSpeed(SPEED);          // set the motor speed.

//  Serial.begin(9600);                //for debuging.
}

{
  pinMode(LEDL_PIN, OUTPUT);    //Set the Left turn LED pin as an output
  pinMode(LEDR_PIN, OUTPUT);    //Set the Right turn LED pin as an output
  pinMode(LEDLF_PIN, OUTPUT);   //Set the Left Light LED pin as an output
  pinMode(LEDRF_PIN, OUTPUT);   //Set the Left Light LED pin as an output
}
}
void loop(){
{
  int val = analogRead(POTFR);        //get the potentiometer value (range 0-1023)
  int val2 = analogRead(POTRR);       //get the potentiometer value (range 0-1023)
  val= map(val,0,1023,-100,100);      // map pot range in the stepper range.
  val2= map(val2,0,1023,-100,100);    // map pot range in the stepper range.

/*  int analog_in = analogRead(POTFR);
  stepper.moveTo(analog_in);
  stepper.setSpeed(100);
  stepper.runSpeedToPosition();
*/  
        if(abs(val - val2)> NOISE){   //if diference is greater than 2 steps.
      if(newpos = (val - val2));
      if(newpos > pos){
          stepper.moveTo(-newpos);   // move one step to the left.
//          stepper.run();

//          val = millis();
//          val2 = millis();
          pos++;
          }
//          stepper1.run();
      if(newpos = (val - val2));
      if(newpos < pos){
          stepper.moveTo(-newpos);    // move one step to the right.
//          stepper.run();

//          val = millis();
//          val2 = millis();
          pos--;
         }
      }
          stepper.run();

//  Serial.println(pos);      //for debuging...
//  Serial.println(val);
//  Serial.println(val2);
//  Serial.println(newpos);
//  delay(100);

  CheckTimeout();   //check for inactivity.

}

  ms = millis();                //record the current time
  myBtnL.read();                //Read Left button
  myBtnR.read();                //Read Right button

  switch (STATE) {

    //This state watches for short and long presses, switches the LED for
    //short presses, and moves to the TO_BLINK state for long presses.
    case ONOFF:{
    if (myBtnL.isPressed()){                    //button pressed
        BlinkL();                               //so blink Left turn LED
      }
      if (myBtnL.wasPressedFor(SHORT_PRESS)){   //if button pressed for short time,
        fastBlinkL();                           //blink Left turn LED 3 times.
      }
      else if (myBtnL.pressedFor(LONG_PRESS)){  //if button pressed for long time,
        STATE = TO_BLINK;                       //go to the next state.
        }
  }
      {
        if (myBtnR.isPressed()){
        BlinkR();
      }
      if (myBtnR.wasPressedFor(SHORT_PRESS)){
        fastBlinkR();
      }
      else if (myBtnR.pressedFor(LONG_PRESS)){
        STATE = TO_BLINK;
        }
      }
       break;

    //This is a transition state where we start the fast blink as feedback to the user,
    //but we also need to wait for the user to release the button, i.e. end the
    //long press, before moving to the BLINK state.
    case TO_BLINK:
      if (myBtnL.wasReleased()){
        digitalWrite(LEDL_PIN, LOW);
        ledStateL = false;
      }
      if (myBtnR.wasReleased()){
        digitalWrite(LEDR_PIN, LOW);
        ledStateR = false;
      }
      else
        STATE = BLINK;
      break;

    //The fast-blink state. Watch for another long press which will cause us to
    //turn the LED off (as feedback to the user) and move to the TO_ONOFF state.
    case BLINK:
      if (myBtnL.isPressed()) {
        for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=10){
        analogWrite(LEDLF_PIN, fadeValue);
        delay(0);         // wait for 0 milliseconds to see the dimming effect
      }
//        switchFadeLin();
        BlinkL();          // we still need blinking, while side light on
      }
       if (myBtnR.isPressed()) {
        for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=10){
        analogWrite(LEDRF_PIN, fadeValue);
        delay(0);
      }
//          switchFadeRin();
        BlinkR();          // we still need blinking, while side light on
      }
      else
      STATE = TO_ONOFF;
      break;

    //This is a transition state where we just wait for the user to release the button
    //before moving back to the ONOFF state.
    case TO_ONOFF:
      if (myBtnL.wasReleased()){
        delay(2000);
        for(int fadeValue = 255 ; fadeValue <= 0; fadeValue -=50){
        analogWrite(LEDLF_PIN, fadeValue);
        delay(30);
//        analogWrite(LEDLF_PIN, LOW);
//        switchFadeLout();
//        ledStateL = false;
//        STATE = ONOFF;
      }
      }
        if (myBtnR.wasReleased()){
        delay(2000);
        for(int fadeValue = 255 ; fadeValue <= 0; fadeValue -=50){
        analogWrite(LEDRF_PIN, fadeValue);
        delay(30);
//        analogWrite(LEDLF_PIN, LOW);
//        switchFadeLout();
//        ledStateL = false;
//        STATE = ONOFF;
        }
        }
      if (myBtnR.isReleased()){
        analogWrite(LEDRF_PIN, LOW);
        digitalWrite(LEDR_PIN, LOW);
      STATE = ONOFF;
       }
      if (myBtnL.isReleased()){
        analogWrite(LEDLF_PIN, LOW);
        digitalWrite(LEDL_PIN, LOW);
      STATE = ONOFF;
      }
      break;

  }

}

//-----Reverse the current LED state. If it's on, turn it off. If it's off, turn it on.-------
void switchLEDL()
{
  msLast = ms;                 //record the last switch time
  ledStateL = !ledStateL;
  //    ledState = false;
  digitalWrite(LEDL_PIN, ledStateL);
}

void switchLEDR()
{
  msLast = ms;                 //record the last switch time
  ledStateR = !ledStateR;
  //    ledState = false;
  digitalWrite(LEDR_PIN, ledStateR);
}

//------------------Blink with the turn LED "myResult" times.---------------------------------
void fastBlinkL()
{
  for (int counter = 1; counter <= myResult; counter = counter + 1) {
    digitalWrite(LEDL_PIN, HIGH);
    delay(BLINK_INTERVAL);
    digitalWrite(LEDL_PIN, LOW);
    delay(BLINK_INTERVAL);
  }
  ledStateL = false;
}
void fastBlinkR()
{
  for (int counter = 1; counter <= myResult; counter = counter + 1) {
    digitalWrite(LEDR_PIN, HIGH);
    delay(BLINK_INTERVAL);
    digitalWrite(LEDR_PIN, LOW);
    delay(BLINK_INTERVAL);
  }
  ledStateR = false;
}

//------------------------------Blink with the LED.-------------------------------------------
void BlinkL()
{
    digitalWrite(LEDL_PIN, HIGH);
    delay(BLINK_INTERVAL);
    digitalWrite(LEDL_PIN, LOW);
    delay(BLINK_INTERVAL);
  
  ledStateL = false;
}
void BlinkR()
{
    digitalWrite(LEDR_PIN, HIGH);
    delay(BLINK_INTERVAL);
    digitalWrite(LEDR_PIN, LOW);
    delay(BLINK_INTERVAL);
  
  ledStateR = false;
}

//--------------------------Turno ON the LED with fade effect.------------------------------
void switchFadeLin()
{
  analogWrite(LEDLF_PIN, brightnessL);            // set the brightness of pin 9:
  brightnessL = brightnessL + fadeAmountL;          // change the brightness for next time through the loop
  if (brightnessL == 0 || brightnessL == 255); 
//  {    // reverse the direction of the fading at the ends of the fade
//    fadeAmountL = -fadeAmountL ;
//  }
//  delay(30);          // wait for 30 milliseconds to see the dimming effect
 
}
void switchFadeRin()
{
  analogWrite(LEDLF_PIN, brightnessL);            // set the brightness of pin 9:
  brightnessL = brightnessL + fadeAmountL;          // change the brightness for next time through the loop
  if (brightnessL == 0 || brightnessL == 255); 
//  {    // reverse the direction of the fading at the ends of the fade
//    fadeAmountL = -fadeAmountL ;
// }
//  delay(30);          // wait for 30 milliseconds to see the dimming effect
 
}

//--------------------------Turn off the LED with fade effect.--------------------------------
void switchFadeLout()
{
  analogWrite(LEDLF_PIN, brightnessL);            // set the brightness of pin 9:
  brightnessL = brightnessL + fadeAmountL;          // change the brightness for next time through the loop
  if (brightnessL == 0 || brightnessL == 255); 
  {    // reverse the direction of the fading at the ends of the fade
    fadeAmountL = -fadeAmountL ;
  }
  delay(30);          // wait for 30 milliseconds to see the dimming effect
        ledStateL = false;
}

void switchFadeRout()
{
  analogWrite(LEDRF_PIN, brightnessR);            // set the brightness of pin 9:
  brightnessR = brightnessR + fadeAmountR;          // change the brightness for next time through the loop
  if (brightnessR == 0 || brightnessR == 255); 
  {    // reverse the direction of the fading at the ends of the fade
    fadeAmountR = -fadeAmountR ;
  }
  delay(30);        // wait for 30 milliseconds to see the dimming effect
        ledStateR = false;
}

