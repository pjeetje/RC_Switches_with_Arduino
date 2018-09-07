#include <SoftwareSerial.h>
// this program works with two rc channels connected to two interupts.
// you could us it for blinking lights/ brake lights, but also for other leds, the used channel does'nt matter
// there are eight possible outputs, and a arduino compatible mp3 player that could be connected
// it works by making decisions dependant of the pulse with given to one of the interupts 
// blinkLed , switchModi and switchA, switchB, switchC, switchD and mp3Command are possible subroutines to call
// switches have different subroutines to save the stat where they are in, a switch (led) can be on or off
// switchA through switchD work with a pulse, one pulse set switch on, after 1 sec gives the same pulse an off command and vice versa
// blinkled works as long as the pulse is available
// the switchModi is an alternative, each pulse toggles through different modes in a cyclic way.
// also an mp3 player is connected, that could play a sound
//thanks to Duane http://rcarduino.blogspot.com/      

#define CMD_SET_VOLUME    0X06  // for mp3 player
#define CMD_SEL_DEV       0X09  // for mp3 player
#define CMD_RESET         0X0C  // for mp3 player
#define CMD_PLAY          0X0D  // for mp3 player
#define CMD_PLAY_W_VOL    0X22  // for mp3 player
#define DEV_TF            0X02  // for mp3 player
#define CMD_PLAY_INDEX    0X03  // for mp3 player


volatile int           nRC1In = 1500; // volatile, we set this in the Interrupt,read it in loop so it must be declared volatile,value neutral RC1
volatile int           nRC2In = 1500; // volatile, we set this in the Interrupt,read it in loop so it must be declared volatile,value neutral RC2
volatile unsigned long ulStartPeriodRC1 = 0;    // set in the interrupt
volatile unsigned long ulStartPeriodRC2 = 0;    // set in the interrupt
volatile boolean       bNewRC1Signal = false;   // set in the interrupt and read in the loop
volatile boolean       bNewRC2Signal = false;   // set in the interrupt and read in the loop
// Pin 2 and Pin 3 are used as interrupt pins
// Pin 4 and Pin 13 are used for Softwareserial


const int     RC1_SIGNAL_IN=0;      // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
const int     RC2_SIGNAL_IN=1;      // INTERRUPT 1 = DIGITAL PIN 3 - use the interrupt number in attachInterrupt
const int     RC1_SIGNAL_IN_PIN=2;  // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead
const int     RC2_SIGNAL_IN_PIN=3;  // INTERRUPT 1 = DIGITAL PIN 3 - use the PIN number in digitalRead
const int     ARDUINO_RX= 4;        // should connect to TX of the Serial MP3 Player module
const int     ARDUINO_TX =13;       // connect to RX of the module
const int     ledPin5 = 5;
const int     ledPin6 = 6;
const int     ledPin7 = 7;
const int     ledPin8 = 8;
const int     ledPin9 = 9;
const int     ledPin10 = 10;
const int     ledPin11 = 11;
const int     ledPin12 = 12;             // attached to 12V module 
const int     analogInPin = A2;
const long    switchinterval = 1000;     // switch may be toggled after 1 sec
const long    blinkinterval = 250;       // blinking interval
const long    flashinterval_on = 80;     // flash interval
const long    flashinterval_off = 1060;  // flash  interval period between
unsigned long blinkMillis;               // variable for blinking time
unsigned long oldblinkMillis = 0;        // variable for blinking time
unsigned long flashMillis_on;            // variable for blinking time
unsigned long oldflashMillis_on = 0;     // variable for blinking time
unsigned long flashMillis_off;           // variable for blinking time
unsigned long oldflashMillis_off = 0;    // variable for blinking time
unsigned long switchMillisA=millis(); // variable for switch time
unsigned long oldswitchMillisA = 0;   // variable for switch time
unsigned long switchMillisB=millis(); // variable for switch time
unsigned long oldswitchMillisB = 0;   // variable for switch time
unsigned long switchMillisC=millis(); // variable for switch time
unsigned long oldswitchMillisC = 0;   // variable for switch time
unsigned long switchMillisD=millis(); // variable for switch time
unsigned long oldswitchMillisD = 0;   // variable for switch time

int           modi=1;               // initial state for a switch
int           mp3choice=1; 
int           flash=LOW;            // initial state for a switch 
int           blinkState = LOW;     //state if led is on or off
int           flashState_on = LOW;  //state if led is on or off
int           flashState_off = LOW; //state if led is on or off
int           switchStateA = HIGH;  // state of switch, possible to change the switch state after 1 sec (switchinterval)
int           ledStateA=HIGH;       //state if led is on or off
int           ledStateB=HIGH;       //state if led is on or off
int           ledStateC=HIGH;       //state if led is on or off
int           ledStateD=HIGH;       //state if led is on or off

static int8_t Send_buf[8] = {0};      // variable for mp3 player Buffer for Send commands.  // BETTER LOCALLY
static int8_t pre_vol, volume = 0x0f; //variable for mp3 player Volume. 0-30 DEC values. 0x0f = 15. 
int val = 0;       // variable to store the value coming from the sensor



SoftwareSerial mp3(ARDUINO_RX, ARDUINO_TX);

void setup()
{
  //these pins are available for leds, 
  // pin 2 and 3 are interupts, pin 4 and 13 are for mp3 player
  pinMode(ledPin5,  OUTPUT);
  pinMode(ledPin6,  OUTPUT);
  pinMode(ledPin7,  OUTPUT);
  pinMode(ledPin8,  OUTPUT);
  pinMode(ledPin9,  OUTPUT); 
  pinMode(ledPin10, OUTPUT);
  pinMode(ledPin11, OUTPUT);
  pinMode(ledPin12, OUTPUT);
  // we want the function calcInputRC1 or calcinputRC2 to be called whenever INT0 or INT1 changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  attachInterrupt(RC1_SIGNAL_IN, calcInputRC1, CHANGE);
  attachInterrupt(RC2_SIGNAL_IN, calcInputRC2, CHANGE);
  // initialize mp3 player
  mp3.begin(9600);
  delay(500);
  mp3Command(CMD_SEL_DEV,DEV_TF);  
  delay(200);
  val = analogRead(analogInPin);
  volume=map(val, 0, 1023, 0, 31);
  mp3Command(CMD_SET_VOLUME,volume);
  delay(200);
}

void loop()
{
  // if a new RC1 or RC2 signal has been measured, a Interrupt has occured
  // switching is possible by giving a pulse , led turns on, after 1 sec it is possible to give same pulse to turn led off
  // another possibility is to work with modi, each pulse change the modi and after X pulses starts again
  
  if (bNewRC1Signal or bNewRC2Signal)
  {
    //all switching from interupt 0 (RC1)
    if (nRC1In > 1800)
    {
      switchA(ledPin12);
    }
    else if (nRC1In < 1200)
    {
      switchB(ledPin11);
    }
    else if (nRC1In > 1450 && nRC1In < 1550 )
    {
      // do nothing
    }
    
    // all switching from interupt 1 (RC2)
    if (nRC2In < 1200) 
    {
      // another possibility is to work with modi, each pulse change the modi and after X pulses starts again
      switchModi(ledPin10,ledPin9);
    }
    else if (nRC2In > 1800) 
    {
      mp3switch();
    }
    
    // set this back to false when we have finished, else calcInput will not update nRC1In or nRC2In
    bNewRC1Signal = false;
    bNewRC2Signal = false;
  }
  // if flashing is wanted has to be done in loop
  if (flash==HIGH)
  {
    flashLed(ledPin10,ledPin9);
  }
}

void switchModi(int a, int b){
  // switchmodi gives possabillity for 4 modi (or more) toggling with each pulse
  if (switchStateA==HIGH){
    //do something according to the modi
    switch (modi){
      case 0:
      flash=LOW;
      //led off
      digitalWrite(a, LOW);
      digitalWrite(b, LOW);
      modi=modi+1;
      break;
      case 1:
      //do something when var equals 1
      digitalWrite(a, HIGH);
      digitalWrite(b, HIGH);
      modi=modi+1;
      break;
      case 2:
      //do something when var equals 1
      digitalWrite(a, HIGH);
      digitalWrite(b, LOW);
      modi=modi+1;
      break;
      case 3:
      //do something when var equals 1
      digitalWrite(a, LOW);
      digitalWrite(b, HIGH);
      modi=modi+1;
      break;
      case 4:
      //do something when var equals 1
      digitalWrite(a, LOW);
      digitalWrite(b, HIGH);
      modi=0;
      flash=HIGH;
      break;
      default: 
      // if nothing else matches, do the default
      // default is optional
      break;
    } 
    switchStateA = LOW;
  }
  switchMillisA = millis();
  
  if (switchMillisA - oldswitchMillisA >= switchinterval) {
    switchStateA = HIGH;
    oldswitchMillisA=switchMillisA;
  }    
} 

void blinkLed(int a)
{  blinkMillis = millis();
  if (blinkMillis - oldblinkMillis >= blinkinterval) 
  {
    oldblinkMillis = blinkMillis;
    blinkState= !blinkState;
    digitalWrite(a, blinkState);
  }
}

void flashLed(int a,int b)
{   flashMillis_off = millis();
  flashMillis_on = millis();
  if (flashMillis_off - oldflashMillis_off >= flashinterval_off) 
  {
    oldflashMillis_off = flashMillis_off;
    flashState_off = !flashState_off;
  }
  if  (flashState_off==HIGH)
  {
    if (flashMillis_on - oldflashMillis_on >= flashinterval_on) 
    {
      oldflashMillis_on = flashMillis_on;
      flashState_on= !flashState_on;
      digitalWrite(a, flashState_on);
      digitalWrite(b, !flashState_on);
    }
  }
}

void switchA(int b){
  //timing used otherwise switch reacts to fast
  // switch time for a possible toggle 1 sec (switchinterval)
  
  switchMillisA = millis();
  
  if (switchMillisA - oldswitchMillisA >= switchinterval) 
  {
    digitalWrite(b, ledStateA);
    ledStateA = !ledStateA;
    oldswitchMillisA=switchMillisA;
  }
} 

void switchB(int b){
  //timing used otherwise switch reacts to fast
  // switch time for a possible toggle 1 sec (switchinterval)
  switchMillisB = millis();
  
  if (switchMillisB - oldswitchMillisB >= switchinterval) 
  {
    digitalWrite(b, ledStateB);
    ledStateB = !ledStateB;   
    oldswitchMillisB=switchMillisB;
  }
} 

void mp3switch()
{
  //timing used otherwise switch reacts to fast
  // switch time for a possible toglle 1 sec (switchinterval)
  switchMillisC = millis();
  if (switchMillisC - oldswitchMillisC >= switchinterval) 
  {
    oldswitchMillisC=switchMillisC;
    ledStateC = !ledStateC;
    if (ledStateC == LOW) 
    {
      mp3Command(CMD_PLAY_INDEX, 0X0001);
      } else 
      {
        mp3Command(CMD_PLAY_INDEX, 0X0002);
      }
    }
  } 
  
  void switchD(int b){
    //timing used otherwise switch reacts to fast
    // switch time for a possible toggle 1 sec (switchinterval)
    switchMillisD = millis();
    
    if (switchMillisD - oldswitchMillisD >= switchinterval) 
    {
      digitalWrite(b, ledStateD);
      ledStateD = !ledStateD;   
      oldswitchMillisD=switchMillisD;
    }
  } 
  
  
  void calcInputRC1()
  {
    // if the pin is high, its the start of an interrupt
    if (digitalRead(RC1_SIGNAL_IN_PIN) == HIGH)
    {
      // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
      // easy to understand and works very well
      ulStartPeriodRC1 = micros();
    }
    else
    {
      // if the pin is low, its the falling edge of the pulse so now calculate the pulse duration by subtracting the
      // start time ulStartPeriod from the current time returned by micros()
      if (ulStartPeriodRC1 && (bNewRC1Signal == false))
      {
        nRC1In = (int)(micros() - ulStartPeriodRC1);
        ulStartPeriodRC1 = 0;
        // tell loop we have a new signal on the RC1 channel
        // we will not update nRC1In until loop sets bNewRC1Signal back to false
        bNewRC1Signal = true;
      }
    }
  }
  
  void calcInputRC2()
  {
    // if the pin is high, its the start of an interrupt
    if (digitalRead(RC2_SIGNAL_IN_PIN) == HIGH)
    {
      // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
      // easy to understand and works very well
      ulStartPeriodRC2 = micros();
    }
    else
    {
      // if the pin is low, its the falling edge of the pulse so now calculate the pulse duration by subtracting the
      // start time ulStartPeriod from the current time returned by micros()
      if (ulStartPeriodRC2 && (bNewRC2Signal == false))
      {
        nRC2In = (int)(micros() - ulStartPeriodRC2);
        ulStartPeriodRC2 = 0;
        // tell loop we have a new signal on the RC1 channel
        // we will not update nRC1In until loop sets bNewRC1Signal back to false
        bNewRC2Signal = true;
      }
    }
  }
  
  void mp3Command(int8_t command, int16_t dat)
  {
    delay(20);
    Send_buf[0] = 0x7e;   
    Send_buf[1] = 0xff;   
    Send_buf[2] = 0x06;    
    Send_buf[3] = command;
    Send_buf[4] = 0x00;   // 0x00 NO, 0x01 feedback
    Send_buf[5] = (int8_t)(dat >> 8);  //datah
    Send_buf[6] = (int8_t)(dat);       //datal
    Send_buf[7] = 0xef;   //
    for(uint8_t i=0; i<8; i++)
    {
      mp3.write(Send_buf[i]) ;
    }
  }
  

