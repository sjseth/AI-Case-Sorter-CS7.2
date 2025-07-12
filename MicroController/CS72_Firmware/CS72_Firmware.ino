/// VERSION CS 7.2.250711.4.1 ///
/// REQUIRES AI SORTER SOFTWARE VERSION 1.1.48 or newer

#include <Wire.h>

//You may need to add the TMCStepper library to your Arduino IDE.
//In the Sketch menu, select Include Library -> Manage Libraries
//In the library manager search for "TMCStepper". Find the TMCStepper library by teemuatlut and click the Install button. 
//You may have to close and reopen the Arduino IDE before the library is recognized
//at this time of this firmware version we are using TMCStepper 0.7.3.
#include <TMCStepper.h>
#include <SoftwareSerial.h>   

#define FIRMWARE_VERSION "7.2.250711.4.1"

#define CASEFAN_PWM 9 //controls case fan speed
#define CASEFAN_LEVEL 100 //0-100 
#define CASEFAN_SW_CTRL false //whether speed controls show in the software

#define CAMERA_LED_PWM 11 //the output pin for the digital PWM 
#define CAMERA_LED_LEVEL  200 //camera brightness if using digital PWM, otherwise ignored 

#define FEED_SENSOR 10 //the proximity sensor under the feed wheel
#define FEED_DIRPIN 8 //DIRECTION signal for the feed motor direction
#define FEED_STEPPIN 7 //PULSE signal for the feed motor steps
#define FEED_ENABLE A0 //Feed motor enable controll pin

//UART COMMS
#define FEED_UART_RX A4 //FEED UART RX COMMS 
#define FEED_UART_TX A5 //FEED UART TX COMMS 
#define FEED_IN_REVERSE false //set to true to reverse direction of feed motor. 
#define FEED_MICROSTEPS 16  //how many microsteps the controller is configured for. 
#define FEED_HOMING_SENSOR A3  //connects to the feed wheel homing sensor
#define FEED_HOMING_SENSOR_TYPE 0 //1=NO (normally open) default switch, 0=NC (normally closed) (optical switches) 
#define FEEDSENSOR_ENABLED true //enabled if feedsensor is installed and working;//this is a proximity sensor under the feed tube which tells us a case has dropped completely
#define FEEDSENSOR_TYPE 1 // NPN = 0 (default), PNP = 1
#define FEED_HOMING_ENABLED true //enabled feed homing sensor
#define FEED_HOMING_OFFSET_STEPS 5 //additional steps to continue after homing sensor triggered
#define FEED_STEPS 60  //The amount to travel before starting the homing cycle. Should be less than (80 - FEED_HOMING_OFFSET_STEPS)
#define FEED_OVERSTEP_THRESHOLD 140 //if we have gone this many steps without hitting a homing node, something is wrong. Throw an overstep error
#define FEED_DONE_SIGNAL 12   // Writes HIGH Signal When Feed is done. Used for mods like AirDrop

//FEED MOTOR SPEED / ACCELLERATION SETTINGS (DISABLED BY DEFAULT)
#define FEED_MOTOR_SPEED 90 //range of 1-100
#define FEED_ACC_SLOPE 32  //2 steps * 16 MicroStes
#define ACC_FEED_ENABLED false //enabled or disables feed motor accelleration. 


#define SORT_DIRPIN 3 //DIRECTION signal for the sorter motor
#define SORT_STEPPIN 2 //PULSE signal for the sorter motor
#define SORT_UART_RX 5 //SORT UART RX COMMS
#define SORT_UART_TX 6 //SORT UART TX COMMS
#define SORT_ENABLE 4 //SORT motor enable control
#define SORT_IN_REVERSE false //reverse direction of sorter motor
#define SORT_MICROSTEPS 16 //how many microsteps the controller is configured for. 
#define SORT_HOMING_SENSOR A2  //connects to the sorter homing sensor
#define SORT_HOMING_SENSOR_TYPE 0 //1=NO (normally open) default switch, 0=NC (normally closed) (optical switches)
#define SORT_HOMING_ENABLED true //home sorter on startup and 0
#define SORT_HOMING_OFFSET_STEPS 0 //additional steps to continue after homing sensor triggered
#define SORT_MOTOR_SPEED 94 //range of 1-100
//SORT MOTOR SPEED / ACCELLERATION SETTINGS (ENABLED BY DEFAULT)
#define ACC_SORT_ENABLED true // default true
#define ACC_FACTOR 1200 //1200 is default factor
#define SORT_ACC_SLOPE 64 //64 is default - slope this is the number of microsteps to accelerate and deaccellerate in a sort. 

//STEPPER MOTOR UART SETTINGS
#define R_SENSE 0.11f 
#define DRIVER_ADDRESS 0b00 
#define FEED_CURRENT 900 //mA - 1100 is default 1amp. 1100=1.1amp, 900=.9amp, etc
#define SORT_CURRENT 900 //mA - 1000 is default 1.1 amp.

//AIRDROP / 12v signaling
#define AIR_DROP_ENABLED false //enables airdrop

#define AUTO_MOTORSTANDBY_TIMEOUT 0 // 0 = disabled; The time in seconds to wait after no motor movement before putting motors in standby

//ARDUINO CONFIGURATIONS
//number of steps between chutes. With the 8 and 10 slot attachments, 20 is the default. 
//If you have customized sorter output drops, you will need to change this setting to meet your needs. 
//Note there are 200 steps in 1 revolution of the sorter motor. 
#define SORTER_CHUTE_SEPERATION 20 


//FEED DELAY SETTINGS

// Used to send signal to add-ons when feed cycle completes (used by airdrop mod). 
// IF NOT USING MODS, SET TO 0. With Airdrop set to 60-100 (length of the airblast)
#define FEED_CYCLE_COMPLETE_SIGNALTIME 100 

// The amount of time to wait after the feed completes before sending the FEED_CYCLE_COMPLETE SIGNAL
// IF NOT USING MODS, SET TO 0. with Airdrop set to 30-50 which allows the brass to start falling before sending the blast of air. 
#define FEED_CYCLE_COMPLETE_PRESIGNALDELAY 30

// Time in milliseconds to wait before sending "done" response to serialport (allows for everything to stop moving before taking the picture): runs after the feed_cycle_complete signal
// With AirDrop mod enabled, it needs about 20-30MS. If airdrop is not enabled, it should be closer to 50-70. 
// If you are getting blurred pictures, increase this value. 
#define FEED_CYCLE_NOTIFICATION_DELAY 120 

//when airdrop is enabled, this value is used instead of SLOT_DROP_DELAY but does the same thing
//Usually can be 100 or lower, increase value if brass not clearing the tube before it moves to next slot. 
#define FEED_CYCLE_COMPLETE_POSTDELAY 100

// number of MS to wait after feedcycle before moving sort arm.
// Prevents slinging brass. 
// This gives time for the brass to clear the sort tube before moving the sort arm. 
#define SLOT_DROP_DELAY 550

//DEBOUNCE is a feature to counteract case bounce which can occur if the machine runs out of brass and a peice of brass drops a distance from
//from the collator to the feeder. It developes speed and bounces of the prox sensor triggering the sensor and bouncing back up to cause a jam. 
//this seeks to eliminate that by adding a small pause to let the case bounce and settle. 

#define DEBOUNCE_TIMEOUT 300 //default 500. The number of milliseconds without sensor activation (meaning no brass in the feed) required to trigger a debounce pause.

#define DEBOUNCE_PAUSE_TIME 500 //default 500.  Set to 0 to disable. The number of milliseconds to pause to wait for case to settle. 

///END OF USER CONFIGURATIONS ///
///DO NOT EDIT BELOW THIS LINE ///
int cameraLEDLevel = CAMERA_LED_LEVEL; 
int caseFanLevel = CASEFAN_LEVEL;
double fanPercentConversion=0;
int notificationDelay = FEED_CYCLE_NOTIFICATION_DELAY;
bool airDropEnabled = AIR_DROP_ENABLED;
int feedCycleSignalTime = FEED_CYCLE_COMPLETE_SIGNALTIME;
int feedCyclePreDelay = FEED_CYCLE_COMPLETE_PRESIGNALDELAY;
int feedCyclePostDelay = FEED_CYCLE_COMPLETE_POSTDELAY;
int slotDropDelay = SLOT_DROP_DELAY;
int dropDelay =  airDropEnabled ? feedCyclePostDelay : slotDropDelay;
int feedDirection = FEED_IN_REVERSE == false;
long autoMotorStandbyTimeout = AUTO_MOTORSTANDBY_TIMEOUT;

int feedCurrent = FEED_CURRENT;
int sortCurrent = SORT_CURRENT;
bool enableSftCurrCtrl = true;

int feedSpeed = FEED_MOTOR_SPEED; //represents a number between 1-100
int feedSteps = FEED_STEPS;
int feedMotorSpeed = 500;//this is default and calculated at runtime. do not change this value

int sortSpeed = SORT_MOTOR_SPEED; //represents a number between 1-100
int sortSteps = SORTER_CHUTE_SEPERATION;
int sortMotorSpeed = 500;//this is default and calculated at runtime. do not change this value
int homingSteps = 0;

int feedMicroSteps = feedSteps * FEED_MICROSTEPS;

const int feedramp = (ACC_FACTOR + (FEED_MOTOR_SPEED /2)) / FEED_ACC_SLOPE;
const int sortramp = (ACC_FACTOR + (SORT_MOTOR_SPEED /2)) / SORT_ACC_SLOPE;
int feedOverTravelSteps = feedMicroSteps - (FEED_OVERSTEP_THRESHOLD * FEED_MICROSTEPS);
int feedOffsetSteps = FEED_HOMING_OFFSET_STEPS;
int feedHomingOffset = feedOffsetSteps * FEED_MICROSTEPS;

bool FeedScheduled = false;
bool IsFeeding = false;
bool IsFeedHoming = false;
bool IsFeedHomingOffset = false;
bool FeedCycleInProgress = false;
bool FeedCycleComplete = false;
bool IsFeedError = false;

int FeedSteps = feedMicroSteps;
int FeedHomingOffsetSteps = feedHomingOffset;
int feedDelayMS = 150;
int sortDelayMS = 400;

bool forceFeed=false;
String input = "";
int qPos1 = 0;
int qPos2 = 0;
int sortStepsToNextPosition = 0;
int sortStepsToNextPositionTracker=0;
bool SortInProgress = false;
bool SortComplete = false;
bool IsSorting = false;
bool IsSortHoming = false;

int sortOffsetSteps = SORT_HOMING_OFFSET_STEPS;
int sortHomingOffset = sortOffsetSteps * SORT_MICROSTEPS;
bool IsSortHomingOffset = false;
int SortHomingOffsetSteps = sortHomingOffset;
bool sorterIsHomed = false;

int slotDelayCalc = 0;

bool IsTestCycle=false;
bool IsSortTestCycle=false;
unsigned long testCycleInterval=0;
unsigned long testsCompleted=0;
int sortToSlot=0;
unsigned long theTime;
unsigned long timeSinceLastSortMove;
unsigned long timeSinceLastMotorMove;
unsigned long msgResetTimer;

//debounce variables
unsigned long lastTrigger = millis();
int triggerTimeout = DEBOUNCE_TIMEOUT;
int debounceTime= DEBOUNCE_PAUSE_TIME;
bool proxActivated = false;
bool sensorDelay = false;


TMC2209Stepper sortmotorUART(SORT_UART_RX, SORT_UART_TX, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper feedmotorUART(FEED_UART_RX, FEED_UART_TX, R_SENSE, DRIVER_ADDRESS);

void setup() {
  Serial.begin(9600);
  delay(200);

 
  //enable motor controllers 
  pinMode(FEED_ENABLE, OUTPUT);
  pinMode(SORT_ENABLE, OUTPUT);
  digitalWrite(SORT_ENABLE, LOW);
  digitalWrite(FEED_ENABLE, LOW);


  feedmotorUART.begin();                // Initialize driver
  feedmotorUART.toff(4);                 // Enables driver in software
  sortmotorUART.begin();                // Initialize driver
  sortmotorUART.toff(4);                 // Enables driver in software
  delay(500);

  //feedmotorUART.shaft(true);				
  feedmotorUART.rms_current(FEED_CURRENT);       // Set motor RMS current
  feedmotorUART.microsteps(FEED_MICROSTEPS); 
  feedmotorUART.pwm_autoscale(true);
  feedmotorUART.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle
  feedmotorUART.ihold(2);

 // feedmotorUART.ihold(4);
  sortmotorUART.rms_current(SORT_CURRENT);       // Set motor RMS current
  sortmotorUART.microsteps(SORT_MICROSTEPS); 
  sortmotorUART.pwm_autoscale(true);    // Needed for stealthChop
  sortmotorUART.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle
  sortmotorUART.ihold(2);

    //uint8_t temperature = sortmotorUART.
  //Serial.print(F("SORT microsteps: "));   Serial.println(sortmotorUART.microsteps());
 // Serial.print(F("SORT current: "));   Serial.println(sortmotorUART.rms_current()); 
  //Serial.print(F("SORT Stealth: "));   Serial.println(sortmotorUART.stealth()); 
  
  //Serial.print(F("FEED microsteps: "));   Serial.println(feedmotorUART.microsteps());
  //Serial.print(F("FEED current: "));   Serial.println(feedmotorUART.rms_current()); 
  //Serial.print(F("FEED Stealth: "));   Serial.println(feedmotorUART.stealth()); 
  // DRV_STATUS_t drv_status = driver.DRV_STATUS();

  // Check thermal flags
 // bool overtemperature_prewarning = drv_status.otpw;
 // bool overtemperature_shutdown = drv_status.ot;
//Serial.print(F("SORT temp: "));  Serial.println(temperature);

  setSorterMotorSpeed(SORT_MOTOR_SPEED);
  setFeedMotorSpeed(FEED_MOTOR_SPEED);



  pinMode(FEED_DIRPIN, OUTPUT);
  pinMode(FEED_STEPPIN, OUTPUT);
  pinMode(SORT_DIRPIN, OUTPUT);
  pinMode(SORT_STEPPIN, OUTPUT);

  pinMode(FEED_DONE_SIGNAL, OUTPUT);
  pinMode(FEED_HOMING_SENSOR, INPUT);
  pinMode(SORT_HOMING_SENSOR, INPUT);
  pinMode(FEED_SENSOR, INPUT_PULLUP);

   pinMode(CASEFAN_PWM, OUTPUT);
   pinMode(CAMERA_LED_PWM, OUTPUT);


    adjustCameraLED(cameraLEDLevel);
    adjustFanLevel(caseFanLevel);

  digitalWrite(FEED_DIRPIN, feedDirection);
 
  jogSorter();

  IsFeedHoming=true;
  IsSortHoming=true;
  msgResetTimer = millis();
  
   Serial.print("Ready\n");
}


void loop() {
   checkSerial();
   getProxState();
   runSortMotor();
   onSortComplete();
   scheduleRun();
   checkFeedErrors();
   runFeedMotor();
   homeFeedMotor();
   homeSortMotor();
   serialMessenger();
   onFeedComplete();
   runAux();
   MotorStandByCheck();
}

int FreeMem(){
  extern int __heap_start, *__brkval;
  int v;
  return(int) &v - (__brkval ==0 ? (int) &__heap_start : (int) __brkval);
}

bool commandReady = false;
char endMarker = '\n';
char rc;

void recvWithEndMarker() {
    while (Serial.available() > 0 ) {
        rc = Serial.read();
        delay(1);
        if (rc != endMarker) {
            input += rc;
        }
        else {
            commandReady=true; 
            return;
        }
    }
 }
void resetCommand(){
  input="";
  commandReady=false;
}

void checkSerial(){
  if(FeedCycleInProgress==false && SortInProgress==false && Serial.available()>0){
   
      //input = Serial.readStringUntil('\n');
       recvWithEndMarker();
       
       if(!commandReady){
        return;
       }

      if (input.startsWith("stop")) {
         resetCommand();
          FeedScheduled=false;
          IsFeedHoming=false;
          IsFeedHomingOffset = false;
          IsSortHomingOffset = false;
          FeedCycleComplete=true;
          FeedCycleInProgress = false;
          IsTestCycle=false;
        
         // Serial.print("ok\n");
         return;
      } 
      // Serial.print(input);
       
     //this should be most cases
      if (isDigit(input[0])) {
        moveSorterToNextPosition(input.toInt());
        FeedScheduled = true;
        IsFeeding = false;
        scheduleRun();
        resetCommand();
        return;
      }
      if (input.startsWith("homefeeder")) {
        feedDelayMS=400;
          IsFeedHoming=true;
         Serial.print("ok\n");
         resetCommand();
         return;
      } 
      if (input.startsWith("homesorter")) {
        sortDelayMS=400;
           jogSorter();
        qPos1 = 0;
        qPos2 = 0;
          IsSortHoming=true;
          Serial.print("ok\n");
          resetCommand();
         return;
      } 
     if(input.startsWith("status")){
        Serial.print(F("SORT microsteps: "));   Serial.println(sortmotorUART.microsteps());
        Serial.print(F("SORT current: "));   Serial.println(sortmotorUART.rms_current()); 
        Serial.print(F("SORT Stealth: "));   Serial.println(sortmotorUART.stealth()); 
        
        Serial.print(F("FEED microsteps: "));   Serial.println(feedmotorUART.microsteps());
        Serial.print(F("FEED current: "));   Serial.println(feedmotorUART.rms_current()); 
        Serial.print(F("FEED Stealth: "));   Serial.println(feedmotorUART.stealth()); 
        resetCommand();
        return;
     }
      
       if(input.startsWith("feedmotorcurrent:")){
            input.replace("feedmotorcurrent:", "");
            feedCurrent = input.toInt();
            if(feedCurrent>1800){
              feedCurrent=1800;
            }
          feedmotorUART.rms_current(feedCurrent);       // Set motor RMS current
           Serial.print("ok\n");
         resetCommand();
        return;
       }
      if(input.startsWith("sortmotorcurrent:")){
            input.replace("sortmotorcurrent:", "");
            sortCurrent = input.toInt();
            if(sortCurrent>1800){
              sortCurrent=1800;
            }
          sortmotorUART.rms_current(sortCurrent);       // Set motor RMS current
           Serial.print("ok\n");
         resetCommand();
        return;
       }
      if (input.startsWith("sortto:")) {
          input.replace("sortto:", "");
          moveSorterToPosition(input.toInt());
           Serial.print("ok\n");
           resetCommand();
         return;
      } 

      if (input.startsWith("xf:")) {
          input.replace("xf:", "");
          forceFeed = true;
          moveSorterToNextPosition(input.toInt());
          resetCommand();
          FeedScheduled = true;
          IsFeeding = false;
          scheduleRun();
          return;
      }

      if (input.startsWith("getconfig")) {
        Serial.print("{\"FeedMotorCurrent\":");
        Serial.print(feedCurrent);

        Serial.print(",\"FeedMotorSpeed\":");
        Serial.print(feedSpeed);

        Serial.print(",\"FeedCycleSteps\":");
        Serial.print(feedSteps);

        Serial.print(",\"SortMotorCurrent\":");
        Serial.print(sortCurrent);

        Serial.print(",\"SortMotorSpeed\":");
        Serial.print(sortSpeed);

        Serial.print(",\"SortSteps\":");
        Serial.print(sortSteps);

        Serial.print(",\"NotificationDelay\":");
        Serial.print(notificationDelay);

        Serial.print(",\"SlotDropDelay\":");
        Serial.print(slotDropDelay);

        Serial.print(",\"AirDropEnabled\":");
        Serial.print(airDropEnabled);

        Serial.print(",\"AirDropPostDelay\":");
        Serial.print(feedCyclePostDelay);

        Serial.print(",\"AirDropPreDelay\":");
        Serial.print(feedCyclePreDelay);

        Serial.print(",\"AirDropSignalTime\":");
        Serial.print(feedCycleSignalTime);

        Serial.print(",\"FeedHomingOffset\":");
        Serial.print(feedOffsetSteps);

        Serial.print(",\"SortHomingOffset\":");
        Serial.print(sortOffsetSteps);
        
        Serial.print(",\"AutoMotorStandbyTimeout\":");
        Serial.print(autoMotorStandbyTimeout);

        Serial.print(",\"CaseFanSpeedEnabled\":");
        Serial.print(CASEFAN_SW_CTRL);

        Serial.print(",\"CaseFanLevel\":");
        Serial.print(caseFanLevel);

        Serial.print(",\"CameraLEDLevel\":");
        Serial.print(cameraLEDLevel);

        Serial.print(",\"DebounceTimeout\":");
        Serial.print(triggerTimeout);

        Serial.print(",\"DebouncePauseTime\":");
        Serial.print(debounceTime);
  
        Serial.print("}\n");
        resetCommand();
        return;      
      }

        if (input.startsWith("debounceTimeout:")) {
          input.replace("debounceTimeout:", "");
          triggerTimeout = input.toInt();
          Serial.print("ok\n");
          resetCommand();
          return;
        }

        if (input.startsWith("debounceTime:")) {
          input.replace("debounceTime:", "");
          debounceTime = input.toInt();
          Serial.print("ok\n");
          resetCommand();
          return;
        }


       //set feed speed. Values 1-100. Def 60
      if (input.startsWith("feedspeed:")) {
        input.replace("feedspeed:", "");
        feedSpeed = input.toInt();
        setFeedMotorSpeed(feedSpeed);
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      //set feed homing offset
      if (input.startsWith("feedhomingoffset:")) {
        input.replace("feedhomingoffset:", "");
        feedOffsetSteps = input.toInt(); //3
        feedHomingOffset = feedOffsetSteps * FEED_MICROSTEPS; //48
        FeedHomingOffsetSteps = feedHomingOffset; //48

        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("sorthomingoffset:")) {
        input.replace("sorthomingoffset:", "");
        sortOffsetSteps = input.toInt(); //3
        sortHomingOffset = sortOffsetSteps * SORT_MICROSTEPS; //48
        SortHomingOffsetSteps = sortHomingOffset; //48

        Serial.print("ok\n");
        resetCommand();
        return;
      }

      if (input.startsWith("sortspeed:")) {
        input.replace("sortspeed:", "");
        sortSpeed = input.toInt();
        setSorterMotorSpeed(sortSpeed);
        Serial.print("ok\n");
        resetCommand();
        return;
      }

      //set sort steps. Values 1-100. Def 20
      if (input.startsWith("sortsteps:")) {
        input.replace("sortsteps:", "");
        sortSteps = input.toInt();
        Serial.print("ok\n");
        resetCommand();
        return;
      }

      //set feed steps. Values 1-1000. Def 100
      if (input.startsWith("feedsteps:")) {
        input.replace("feedsteps:", "");
        feedSteps = input.toInt();
        feedMicroSteps = feedSteps * FEED_MICROSTEPS;
        feedOverTravelSteps = feedMicroSteps - (FEED_OVERSTEP_THRESHOLD * FEED_MICROSTEPS);
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("notificationdelay:")) {
        input.replace("notificationdelay:", "");
        notificationDelay = input.toInt();
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("slotdropdelay:")) {
        input.replace("slotdropdelay:", "");
        slotDropDelay = input.toInt();
        dropDelay =  airDropEnabled ? feedCyclePostDelay: slotDropDelay;
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("airdropenabled:")) {
        input.replace("airdropenabled:", "");
        airDropEnabled = stringToBool(input);
        dropDelay =  airDropEnabled ? feedCyclePostDelay: slotDropDelay;
        Serial.print("ok\n");
        resetCommand();
        return;
      }

      if (input.startsWith("airdroppostdelay:")) {
        input.replace("airdroppostdelay:", "");
        feedCyclePostDelay = input.toInt();
        dropDelay =  airDropEnabled ? feedCyclePostDelay: slotDropDelay;
        Serial.print("ok\n");
        resetCommand();
        return;
      }
       if (input.startsWith("airdroppredelay:")) {
        input.replace("airdroppredelay:", "");
        feedCyclePreDelay = input.toInt();
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("airdropdsignalduration:")) {
        input.replace("airdropdsignalduration:", "");
        feedCycleSignalTime = input.toInt();
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("fan:")) {
        input.replace("fan:", "");
        caseFanLevel = input.toInt();
        adjustFanLevel(caseFanLevel);
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("automotorstandbytimeout:")) {
        input.replace("automotorstandbytimeout:", "");
        autoMotorStandbyTimeout = input.toDouble();
        Serial.print("ok\n");
        resetCommand();
        return;
      }
      if (input.startsWith("cameraledlevel:")) {
         input.replace("cameraledlevel:", "");
         adjustCameraLED(input.toInt() );
         //Serial.print("LED: ");
         //Serial.print((float)cameraLEDLevel/255.0*100.0);
         //Serial.print("%\n");
         Serial.print("ok\n");
         resetCommand();
         return;
       }

      if (input.startsWith("test:")) {
        input.replace("test:", "");
        IsTestCycle=true;
        testCycleInterval=input.toFloat();
        testsCompleted=0;
        FeedScheduled=false;
        FeedCycleInProgress=false;
        Serial.print("testing started\n");
        resetCommand();
        return;
      }

       if (input.startsWith("sorttest:")) {
        input.replace("sorttest:", "");
        IsSortTestCycle=true;
        testCycleInterval=static_cast<unsigned long>(input.toInt());
        testsCompleted=0;
        Serial.print("testing started\n");
        resetCommand();
        return;
      }
     


       if (input.startsWith("ping")) {
        //Serial.print(FreeMem());
        resetCommand();
        Serial.print(" ok\n");
        return;
      }
      if (input.startsWith("version")) {
       
        Serial.print(FIRMWARE_VERSION);
        Serial.print("\n");
        resetCommand();
        return;
      }
      resetCommand();
      Serial.print("ok\n");
  }
}

bool stringToBool(String str) {
  str.toLowerCase();
  // Compare the string to "true" and return true if they match
  if(str=="true"){
    return true;
  }
  if(str == "1"){
    return true;
  }
  return false;
  
}

//this method is to run all "other" routines not in the main duty cycles (such as tests)
void runAux(){

  //This runs the feed and sort test if scheduled
  if(IsTestCycle==true&&FeedScheduled==false&&FeedCycleInProgress==false){
    if(testsCompleted<testCycleInterval){
       int slot = random(0,6);
         moveSorterToNextPosition(slot);
        
        FeedScheduled = true;
        IsFeeding = false;
        scheduleRun();
        
        Serial.print(testsCompleted);
        Serial.print(" - ");
        Serial.print(slot);
        Serial.print("\n");
        testsCompleted++;

            
    }else{
      IsTestCycle=false;
      testCycleInterval=0;
      testsCompleted=0;
    }
  }

  //this runs the sorter only test cycles if scheduled
  if(IsSortTestCycle==true&&SortInProgress==false){
    if(testsCompleted<testCycleInterval){
       int slot = random(0,8);
       delay(40);
       Serial.print(testsCompleted);
       Serial.print(" - Sorting to: ");
       Serial.println(slot);
       moveSorterToPosition(slot);
        testsCompleted++;
    }else{
      moveSorterToPosition(0);
      Serial.println("Sort Test Completed");
      IsSortTestCycle=false;
      testsCompleted=0;
      testCycleInterval=0;
    }
  }
}


void moveSorterToNextPosition(int position){
    sortToSlot=position;
    sortStepsToNextPosition = (qPos1 * sortSteps * SORT_MICROSTEPS) - (qPos2 * sortSteps * SORT_MICROSTEPS);
    sortStepsToNextPositionTracker = sortStepsToNextPosition;
    if(sortStepsToNextPosition !=0){
      theTime = millis();
       slotDelayCalc = (dropDelay - (theTime - timeSinceLastSortMove));
       slotDelayCalc = slotDelayCalc > 0? slotDelayCalc : 1;
       if(slotDelayCalc > dropDelay){
         slotDelayCalc=dropDelay;
       }

      // Serial.println(slotDelayCalc);
      delay(slotDelayCalc);
    }
    qPos1 = qPos2;
    qPos2 =position;
    SortInProgress = true;
    SortComplete = false;
    IsSorting = true;
}

void moveSorterToPosition(int position){
    sortToSlot=position;
    sortStepsToNextPosition = (qPos1 * sortSteps * SORT_MICROSTEPS) - (position * sortSteps * SORT_MICROSTEPS);
    sortStepsToNextPositionTracker = sortStepsToNextPosition;
   
  // Serial.println(position);
   //Serial.println(sortStepsToNextPosition);
    qPos1 =position;
    qPos2 =position;
    SortInProgress = true;
    SortComplete = false;
    IsSorting = true;
}

void runSortMotor(){
  if(IsSorting==true){
    
    if(sortStepsToNextPosition==0){
     
      if(qPos1==0){
        homingSteps=0;
         IsSortHoming=true;
      }else{
         IsSorting=false;
         SortComplete = true;
      }
      return;
    }
    setAccSortDelay();
    
    if(sortStepsToNextPosition > 0){
      stepSortMotor(true);
      sortStepsToNextPosition--;
    }
    else {
      stepSortMotor(false);
      sortStepsToNextPosition++;
    }
  }
}
void setAccSortDelay(){
    if(ACC_SORT_ENABLED == false){
      sortDelayMS=sortMotorSpeed;
      return;
    }
    //up ramp
    if((abs(sortStepsToNextPositionTracker) - abs(sortStepsToNextPosition)) < SORT_ACC_SLOPE ){
      sortDelayMS = ACC_FACTOR - ((abs(sortStepsToNextPositionTracker) - abs(sortStepsToNextPosition)) * sortramp);
      if (sortDelayMS < sortMotorSpeed) {
         sortDelayMS = sortMotorSpeed;
      }
     
      return;
    }
  
    //down ramp
    if (abs(sortStepsToNextPositionTracker) - abs(sortStepsToNextPosition) > (abs(sortStepsToNextPositionTracker) - SORT_ACC_SLOPE)){
        sortDelayMS = (SORT_ACC_SLOPE - abs(sortStepsToNextPosition)) * sortramp;
         if (sortDelayMS < sortMotorSpeed) {
             sortDelayMS = sortMotorSpeed;
         }
         if (sortDelayMS > ACC_FACTOR) {
             sortDelayMS = ACC_FACTOR;
         }
         
    }else{
      //normal run speed
      sortDelayMS = sortMotorSpeed;
    }
    
}
bool sortDirection = false;
void stepSortMotor(bool forward){
     sortDirection = forward == SORT_IN_REVERSE;
     digitalWrite(SORT_ENABLE, LOW);
     if(forward==true){
       digitalWrite(SORT_DIRPIN, sortDirection);
     }else{
      digitalWrite(SORT_DIRPIN, sortDirection);
    }
    digitalWrite(SORT_STEPPIN, HIGH);
    delayMicroseconds(10);  //pulse width
    digitalWrite(SORT_STEPPIN, LOW);
    delayMicroseconds(sortDelayMS); //controls motor speed
}
void onSortComplete(){
  if(SortInProgress==true && SortComplete==true){
        SortInProgress=false;
        timeSinceLastSortMove = millis();
        timeSinceLastMotorMove = timeSinceLastSortMove;
       // Serial.println("runscheduled");
  }
}

void checkFeedErrors(){
 if(FeedCycleComplete == false && FeedSteps < feedOverTravelSteps){
      FeedScheduled=false;
      FeedCycleComplete=true;
      IsFeeding=false;
      IsFeedHoming= false;
      IsFeedHomingOffset=false;
      IsFeedError = true;
      FeedCycleInProgress = false;
      Serial.println("error:feed overtravel detected");
 }
}
void serialMessenger(){
  
 return; 

}

void onFeedComplete(){
  if(FeedCycleComplete==true&& IsFeedError==false){
   
    timeSinceLastMotorMove = millis();
   //this allows some time for the brass to start dropping before generating the airblast

    if(airDropEnabled)
    {
      delay(feedCyclePreDelay);
      digitalWrite(FEED_DONE_SIGNAL, HIGH);
      delay(feedCycleSignalTime);
      digitalWrite(FEED_DONE_SIGNAL,LOW);
     
    }
    delay(notificationDelay);
    Serial.print("done\n");
    //Serial.flush();
    FeedCycleComplete=false;
    forceFeed= false;
    return;
  }
  
}

void scheduleRun(){
 
  if(FeedScheduled==true && IsFeeding==false){
    //if(digitalRead(FEED_SENSOR) == FEEDSENSOR_TYPE || forceFeed==true || FEEDSENSOR_ENABLED==false){
      if(readyToFeed()){
      //set run variables
      IsFeedError=false;
      FeedSteps = feedMicroSteps;
      FeedScheduled=false;
      FeedCycleInProgress = true;
      FeedCycleComplete=false;
      IsFeeding=true;
     
    }else{
      theTime = millis();
      if(theTime - msgResetTimer > 1000){
         // Serial.flush();
          Serial.println("waiting for brass");
         
          msgResetTimer = millis();
      }
    // Serial.flush();
     
    }
  }
}



void getProxState(){

  //if the sensor is triggered, update the last trigger time and set the variable proxActivated
  if(digitalRead(FEED_SENSOR) == FEEDSENSOR_TYPE){
      proxActivated=true;
      lastTrigger = millis();
      return;
  }

  //sensor is not triggered, set the offTimer and set the variable to false
  proxActivated=false;
    
  //check to see if the time since last trigger is longer than the timeout, if so set the delay variable. 
  if(millis() - lastTrigger > triggerTimeout){
    sensorDelay = true;
  }
}

bool readyToFeed()
{
  //if feedsensor is not enabled, or it is a forcefeed,  we are always ready!
  if(FEEDSENSOR_ENABLED==false || forceFeed==true){
    return true;
  }

  //if no brass is detected, we are not ready
  if(proxActivated == false){
    return false;
  }

  //sensorDelay is calcualted in the getProxState() state method above. 
  if(sensorDelay){
        delay(debounceTime);
        sensorDelay = false;
        return false;
  }
   return true;
}


void runFeedMotor() {
  if(SortInProgress){
    return;
  }

  if(IsFeeding==true && FeedSteps > 0 )
  {
    setAccFeedDelay();
    stepFeedMotor();
    FeedSteps--;
    return;
  }
  if(IsFeeding==true){
    IsFeeding=false;
    IsFeedHoming = true;
  }
  return;

}

void homeFeedMotor(){
  
  if(IsFeedHoming==true ){
   
    if(FEED_HOMING_ENABLED == false){
      IsFeedHoming=false;
       IsFeedHomingOffset = false;
      FeedCycleComplete=true;
      FeedCycleInProgress = false;
      return;
    }
    
    if (digitalRead(FEED_HOMING_SENSOR) == FEED_HOMING_SENSOR_TYPE) {
      IsFeedHoming=false;
      
     // Serial.println("homed!");
      if(FeedCycleInProgress){ //if we are homing initially, we don't need to apply offsets
          IsFeedHomingOffset = true;
          FeedHomingOffsetSteps = feedHomingOffset;
      }
      return;
    }
    stepFeedMotor();
    FeedSteps--;
    return;
  }

  if(IsFeedHomingOffset == true){
    if(feedHomingOffset == 0)
    {
      IsFeedHomingOffset = false;
      FeedCycleComplete=true;
      FeedCycleInProgress = false;
      return;
    }
    if(IsFeedHomingOffset == true && FeedHomingOffsetSteps > 0){
      stepFeedMotor();
    
     // Serial.println(FeedHomingOffsetSteps);
      FeedHomingOffsetSteps--;
      FeedSteps--;
    }
    else if(IsFeedHomingOffset == true && FeedHomingOffsetSteps<=0){
      IsFeedHomingOffset = false;
      FeedCycleComplete=true;
      FeedCycleInProgress = false;
    }
  }
}


void homeSortMotor(){
  if(IsSortHoming==true && SORT_HOMING_ENABLED == false){
     IsSorting=false;
         SortComplete = true;
         IsSortHoming =false;
         IsSortHomingOffset = false;
         return;
  }
  if(IsSortHoming==true){
     //if a sort is in progress and the arm is moving from any position to zero
     //this code is reached when the steps have been completed to go to zero
     //we are going to check if the homing sensor is not activated (which it should be as we are at zero), 
     //if not, we are going to step the motor until it is or we have reached 200 steps (1 complete turn)
    if(IsSorting==true){
         if(digitalRead(SORT_HOMING_SENSOR)!=SORT_HOMING_SENSOR_TYPE){ //
          if(homingSteps < (200*SORT_MICROSTEPS)){
              stepSortMotor(true);  
              homingSteps++; 
          }
          return;
         }
         IsSorting=false;
         SortComplete = true;
         IsSortHoming =false;
         homingSteps=0;
         return;
    }
    //else if we are not doing an offset move (post homing) and the sensor is not 
    //activated, lets keep moving until it is or we hit 210 homing steps (otherwise we turn indefinitely)
    else if(IsSortHomingOffset != true){
      if(digitalRead(SORT_HOMING_SENSOR)!=SORT_HOMING_SENSOR_TYPE){
          if(homingSteps < (210*SORT_MICROSTEPS)){
              stepSortMotor(true);  
              homingSteps++;          
          }
      }else{ //we are homed! Time to schedule an offset move and reset the homing steps counter. 
        IsSortHomingOffset=true;
        SortHomingOffsetSteps = sortHomingOffset;
        homingSteps = 0;
      }
    }

   //If sort homing offset true, means we are in offset steps
    if(IsSortHomingOffset == true){
      if(sortHomingOffset == 0) //if there are no offset steps, we are done
      {
        IsSortHomingOffset = false;
        IsSortHoming=false;
        SortComplete=true;
        return;
      }
      if(SortHomingOffsetSteps > 0){
        stepSortMotor(true);
        SortHomingOffsetSteps--;
      }
      else if(IsSortHomingOffset == true && SortHomingOffsetSteps<=0){
        IsSortHomingOffset = false;
        IsSortHoming=false;
        SortComplete=true;
        homingSteps=0;
      }
    }
  }


 
}



void stepFeedMotor(){
    digitalWrite(FEED_ENABLE, LOW);
    digitalWrite(FEED_STEPPIN, HIGH);
    delayMicroseconds(3);  //pulse width
    digitalWrite(FEED_STEPPIN, LOW);
    delayMicroseconds(feedDelayMS); //controls motor speed
}

void setAccFeedDelay(){
    if(ACC_FEED_ENABLED == false){
      feedDelayMS=feedMotorSpeed;
      return;
    }
    //up ramp
    if((feedMicroSteps - FeedSteps) < FEED_ACC_SLOPE ){
      feedDelayMS = ACC_FACTOR - ((feedMicroSteps - FeedSteps) * feedramp);
      if (feedDelayMS < feedMotorSpeed) {
         feedDelayMS = feedMotorSpeed;
      }
      //Serial.println(msdelay);
      return;
    }
  
    //down ramp
    if (feedMicroSteps - FeedSteps > (feedMicroSteps - FEED_ACC_SLOPE)){
        feedDelayMS = (FEED_ACC_SLOPE - FeedSteps) * feedramp;
         if (feedDelayMS < feedMotorSpeed) {
             feedDelayMS = feedMotorSpeed;
         }
         if (feedDelayMS > ACC_FACTOR) {
             feedDelayMS = ACC_FACTOR;
         }
         //Serial.println(msdelay);
    }else{
      //normal run speed
      feedDelayMS = feedMotorSpeed;
    }
    
}

void setSorterMotorSpeed(int speed) {
  sortMotorSpeed = setSpeedConversion(speed);
}
void setFeedMotorSpeed(int speed) {
  feedMotorSpeed = setSpeedConversion(speed);
}

int setSpeedConversion(int speed) {
  if (speed < 1 || speed > 100) {
    return 500;
  }

  return 1060 - ((int)(((double)(speed - 1) / 99) * (1000 - 60)) + 60);
}

void MotorStandByCheck(){
  if(SortInProgress || IsFeeding)
    return;
  
  if(autoMotorStandbyTimeout==0)
    return;

  theTime = millis();

  if(theTime - timeSinceLastMotorMove > (autoMotorStandbyTimeout*1000) ) {
     digitalWrite(FEED_ENABLE, HIGH);
     digitalWrite(SORT_ENABLE, HIGH);
  }
}

void adjustCameraLED(int level)
 {
   // Trim to acceptable values
   level > 255 ? 255: level;
   level = level < 0 ? 0 : level;
 
   analogWrite(CAMERA_LED_PWM, level);
   cameraLEDLevel = level;
 }



void adjustFanLevel(int level)
{
  fanPercentConversion = level * 2.55;
  level = fanPercentConversion;
  analogWrite(CASEFAN_PWM, level);
  //caseFanLevel = level;
 }


int js=0;
int jogSteps = 25 * SORT_MICROSTEPS;
void jogSorter(){
    for(js=0;js<jogSteps;js++){
      stepSortMotor(false);
    }
}
