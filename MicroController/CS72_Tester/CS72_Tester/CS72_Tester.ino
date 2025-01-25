#include <AccelStepper.h>
#include <MultiStepper.h>

#include <TMCStepper.h>
#include <SoftwareSerial.h>   


#define SORT_HOME_SENSOR A2
#define SORT_STEP 2
#define SORT_DIR 3
#define SORT_UART_RX 5
#define SORT_UART_TX 6
#define SORT_ENABLE 4

#define FEED_HOME_SENSOR A3
#define FEED_STEP 7
#define FEED_DIR 8
#define FEED_UART_RX A4
#define FEED_UART_TX A5
#define FEED_ENABLE 9
#define FEED_PROX_SENSOR 10

#define CAMERA_DIMMER 11
#define AIR_DROP_SIGNAL 12

#define R_SENSE 0.10f 
#define DRIVER_ADDRESS 0b00 
#define MICROSTEPS 32
#define CURRENT 700
#define PULSEDELAY 4
#define MOTORSPEED 100

TMC2209Stepper sortmotorUART(SORT_UART_RX, SORT_UART_TX, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper feedmotorUART(FEED_UART_RX, FEED_UART_TX, R_SENSE, DRIVER_ADDRESS);



struct Motor {
    int stepPin;
    int dirPin;
    int enablePin;
    int homeSensorPin;
};

Motor sortMotor = {SORT_STEP, SORT_DIR, SORT_ENABLE, SORT_HOME_SENSOR};
Motor feedMotor = {FEED_STEP, FEED_DIR, FEED_ENABLE, FEED_HOME_SENSOR};
AccelStepper feedstepper(AccelStepper::DRIVER, FEED_STEP, FEED_DIR);

void setup(){
  Serial.begin(9600);
  delay(200);
  Serial.println("Initializing...");
  
  pinMode(SORT_HOME_SENSOR, INPUT);
  pinMode(FEED_HOME_SENSOR, INPUT); 
  pinMode(FEED_PROX_SENSOR, INPUT);

  pinMode(CAMERA_DIMMER, OUTPUT);
  pinMode(AIR_DROP_SIGNAL, OUTPUT);

  pinMode(FEED_STEP, OUTPUT);
  pinMode(FEED_DIR, OUTPUT);
  pinMode(FEED_ENABLE, OUTPUT);
  pinMode(SORT_STEP, OUTPUT);
  pinMode(SORT_DIR, OUTPUT);
  pinMode(SORT_ENABLE, OUTPUT);
  digitalWrite(FEED_STEP, LOW);
  digitalWrite(FEED_DIR, LOW);
  digitalWrite(FEED_ENABLE, HIGH);
  digitalWrite(SORT_STEP, LOW);
  digitalWrite(SORT_DIR, LOW);

  digitalWrite(SORT_ENABLE, LOW);
  digitalWrite(FEED_ENABLE, LOW);
  




  feedmotorUART.begin();                // Initialize driver
  feedmotorUART.toff(4);                 // Enables driver in software
  sortmotorUART.begin();                // Initialize driver
  sortmotorUART.toff(4);                 // Enables driver in software
  delay(1000);

  Serial.println("Pre-Config Motor Settings:");
   Serial.println("#######################");
  Serial.print(F("FEED microsteps: "));   Serial.println(feedmotorUART.microsteps());
  Serial.print(F("FEED current: "));   Serial.println(feedmotorUART.rms_current()); 
  Serial.print(F("FEED Stealth: "));   Serial.println(feedmotorUART.stealth()); 
  Serial.print(F("SORT microsteps: "));   Serial.println(sortmotorUART.microsteps());
  Serial.print(F("SORT current: "));   Serial.println(sortmotorUART.rms_current()); 
  Serial.print(F("SORT Stealth: "));   Serial.println(sortmotorUART.stealth()); 
	   Serial.println("#######################");
        Serial.println("");


Serial.println("Setting motor values.");
   Serial.println("#######################");
  feedmotorUART.shaft(true);				
  feedmotorUART.rms_current(CURRENT);       // Set motor RMS current
  feedmotorUART.microsteps(MICROSTEPS); 
  feedmotorUART.pwm_autoscale(true);
  feedmotorUART.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle

  digitalWrite(FEED_ENABLE, LOW);
  Serial.print(F("FEED microsteps: "));   Serial.println(feedmotorUART.microsteps());
  Serial.print(F("FEED current: "));   Serial.println(feedmotorUART.rms_current()); 
  Serial.print(F("FEED Stealth: "));   Serial.println(feedmotorUART.stealth()); 


  sortmotorUART.shaft(true); 
  sortmotorUART.rms_current(CURRENT);       // Set motor RMS current
  sortmotorUART.microsteps(MICROSTEPS); 
  sortmotorUART.pwm_autoscale(true);    // Needed for stealthChop
  sortmotorUART.en_spreadCycle(false);   // false = StealthChop / true = SpreadCycle

  Serial.print(F("SORT microsteps: "));   Serial.println(sortmotorUART.microsteps());
  Serial.print(F("SORT current: "));   Serial.println(sortmotorUART.rms_current()); 
  Serial.print(F("SORT Stealth: "));   Serial.println(sortmotorUART.stealth()); 
   Serial.println("#######################");
      Serial.println("");
Serial.println("");
  digitalWrite(FEED_ENABLE, HIGH);
  digitalWrite(SORT_ENABLE, HIGH);
  
  Serial.print("Ready to begin testing\n");

}


void loop(){
  // digitalWrite(SORT_ENABLE, LOW);
  GuidedTest();
  //TestMotors(sortMotor);

}


void TestMotor(Motor motor, int steps){
  
  int teststeps = steps * MICROSTEPS;
  digitalWrite(motor.dirPin, HIGH);
  Serial.println(motor.enablePin);
  digitalWrite(motor.enablePin, LOW); // Assuming LOW enables the motor
    // Step the motor
    for (int i = 0; i < teststeps; i++) {
       
        digitalWrite(motor.stepPin, HIGH);
        delayMicroseconds(PULSEDELAY); // Adjust delay for speed
        digitalWrite(motor.stepPin, LOW);
        delayMicroseconds(MOTORSPEED);
    }

    Serial.println("reversing...");
   digitalWrite(motor.dirPin, LOW);
    for (int i = 0; i < teststeps; i++) {
       
        digitalWrite(motor.stepPin, HIGH);
        delayMicroseconds(PULSEDELAY); // Adjust delay for speed
        digitalWrite(motor.stepPin, LOW);
        delayMicroseconds(MOTORSPEED);
    }
   digitalWrite(motor.enablePin, HIGH);
}

void GuidedTest(){
   while (true) {
        int theChoice = promptUserInt("Select a number from the list below to begin:\n 1. Full test\n 2. Motor Tests\n 3. Switch Test\n 4. Dimmer Test\n 5. Exit");
        switch (theChoice) {
            case 1: 
                fullTest();
                break;
            case 2: 
                promptUser("Testing Sorter Motor. Press ENTER to continue");
                TestMotor(sortMotor, 400);
                promptUser("Testing Feeder Motor. Press ENTER to continue");
                TestMotor(feedMotor, 400);
                break;
            case 3:
                TestSwitches();
                break;
            case 4:
                promptUser("Testing Camera Dimmer. Press ENTER to continue");
                testDimmer(CAMERA_DIMMER);
                break;
            case 5: 
                Serial.println("Exiting guided test.");
                return; // Exit the function
            default:
                Serial.println("Invalid choice. Please select a valid number from the list.");
                break;
        }
    }

}


void fullTest(){
    promptUser("Testing sort motor. Press ENTER to continue.");
    Serial.println("Started...");
    TestMotor(sortMotor, 400);

    promptUser("Testing feed motor. Press ENTER to continue.");
    Serial.println("Started...");
    TestMotor(feedMotor, 400);

    promptUser("Testing sort homing switch. Press ENTER to continue.");
    testSwitch(SORT_HOME_SENSOR, LOW);

    promptUser("Testing feed homing switch. Press ENTER to continue.");
    testSwitch(FEED_HOME_SENSOR, LOW);

    promptUser("Testing feed proximity switch. Press ENTER to continue.");
    testSwitch(FEED_PROX_SENSOR, LOW);

    promptUser("Testing Camera Dimmer. Press ENTER to continue");
    testDimmer(CAMERA_DIMMER);

    promptUser("Testing Air Drop output. Press ENTER to continue");
    testOutput(AIR_DROP_SIGNAL, 1000, 50);
}

void TestSwitches(){
    promptUser("Testing sort homing switch. Press ENTER to continue.");
    testSwitch(SORT_HOME_SENSOR, LOW);

    promptUser("Testing feed homing switch. Press ENTER to continue.");
    testSwitch(FEED_HOME_SENSOR, LOW);

    promptUser("Testing feed proximity switch. Press ENTER to continue.");
    testSwitch(FEED_PROX_SENSOR, LOW);
}



void testDimmer(int pin) {
    Serial.println("Testing dimmer... Press ENTER to stop.");

    int brightness = 0; // Initial brightness
    int fadeAmount = 5; // Amount to change brightness per step

    while (true) {
        // Set the PWM duty cycle
        analogWrite(pin, brightness);

        // Increment or decrement brightness
        brightness += fadeAmount;

        // Reverse direction at the limits
        if (brightness <= 0 || brightness >= 255) {
            fadeAmount = -fadeAmount; // Reverse the direction of fading
        }

        // Check for user input to stop
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Read input until ENTER
            if (input.length() == 0) { // ENTER pressed without typing anything
                Serial.println("Exiting dimmer test.");
                analogWrite(pin, 0); // Turn off dimmer
                break;
            }
        }

        delay(60); // Adjust delay for fade speed
    }
}



void testSwitch(int pinNumber, bool open) {
    Serial.println("Testing switch... Press ENTER to stop.");
    delay(800);

    while (true) {
        // Read the switch state
        int switchState = digitalRead(pinNumber);

        // Output the switch state
        if (switchState == open) {
            Serial.println("Switch is OPEN");
        } else {
            Serial.println("Switch is CLOSED");
        }

        // Check if the user pressed ENTER
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Read input until ENTER
            if (input.length() == 0) { // ENTER pressed without other characters
                Serial.println("Exiting switch test.");
                break;
            }
        }

        delay(100); // Adjust delay for readability
    }
}



void testOutput(int pinNumber, int intervaldelay, int sigtime) {
    Serial.print("Testing output pin: ");
    Serial.print(pinNumber);
    Serial.print(". Press ENTER to stop.\n");
    delay(1000);

    while (true) {
        // Read the switch state
        Serial.write("\nSetting pin HIGH");
        digitalWrite(pinNumber, HIGH);
        delay(sigtime);
       
         Serial.write("\nSetting pin LOW");
        digitalWrite(pinNumber, LOW);
        delay(intervaldelay);
        
        // Check if the user pressed ENTER
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Read input until ENTER
            if (input.length() == 0) { // ENTER pressed without other characters
                Serial.println("Exiting output test.");
                break;
            }
        }

        delay(500); // Adjust delay for readability
    }
}


void promptUser(const char* message) {
    Serial.println(message);
    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Wait for Enter key
            return; // Exit the loop once Enter is pressed
        }
    }
}

int promptUserInt(const char* message) {
    Serial.println(message);
    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); // Wait for Enter key
            input.trim(); // Remove leading/trailing whitespace
            
            // Validate input: toInt() returns 0 if conversion fails
            bool isValid = true;
            for (unsigned int i = 0; i < input.length(); i++) {
                if (!isDigit(input[i]) && !(i == 0 && input[i] == '-')) {
                    isValid = false;
                    break;
                }
            }

            if (isValid) {
                return input.toInt(); // Convert and return integer
            } else {
                Serial.println("Invalid input. Please enter a valid integer:");
            }
        }
    }
}

void MoveMotor(Motor motor, int steps, int dir) {
    // Enable the motor
 //   digitalWrite(motor.enablePin, LOW); // Assuming LOW enables the motor
    digitalWrite(motor.dirPin, dir);
 // digitalWrite(motor.enablePin, LOW); // Assuming LOW enables the motor
    // Step the motor
    for (int i = 0; i < steps; i++) {
       
        digitalWrite(motor.stepPin, HIGH);
        delayMicroseconds(PULSEDELAY); // Adjust delay for speed
        digitalWrite(motor.stepPin, LOW);
        delayMicroseconds(MOTORSPEED);
    }

    // Disable the motor
    digitalWrite(motor.enablePin, HIGH); // Assuming HIGH disables the motor
}

