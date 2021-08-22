#include <Wire.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h>

//Inputs
const int DIR = 9;                  // DIR on controller to pin 9 on arduino
const int STEP = 10;                // PUL on controller to pin 10 on arduino
const int LCD_RS = 12;              // LCD Register Select Signal on arduino pin 12
const int LCD_E = 11;               // LCD Operation Enable Signal on arduino pin 11
const int LCD_DB4 = 5;              // LCD Data Bus 4 Signal on arduino pin 5
const int LCD_DB5 = 4;              // LCD Data Bus 5 Signal on arduino pin 4
const int LCD_DB6 = 3;              // LCD Data Bus 6 Signal on arduino pin 3
const int LCD_DB7 = 2;              // LCD Data Bus 7 Signal on arduino pin 2
const int DIP_SW1 = 1;              // DIP Switch 1 on arduino pin 1
const int DIP_SW2 = 0;              // DIP Switch 2 on arduino pin 0
const int switchU = 6;              // Upper limit switch
const int switchH = 7;              // Home position in standby
const int startSwitch = 8;          // Start switch, latching
const int BPM = A0;                 // Desired bpm
const int Ratio = A1;               // Desired I/E ratio potentiometer (percentage of bag compression, from UofT code)
const int VolumeT = A2;             // Desired tidal volume of compression potentiometer

//Outputs
int buzzer = 13;                    //Buzzer 


// Constants - Variables to edit configurations below, you can tweak some settings here
const int LCD_COLS = 20;            // Number of columns on LCD display
const int LCD_ROWS = 4;             // Number of rows on LCD display
const float ACCEL = 50000.0;        // Acceleration and deceleration rate for stepper motor (steps per second per second)
const float MAXSPEED = 100000.0;     // Set the maximum allowable speed of the stepper motor (steps per second) This is set in setup() function and not used after that
const float HOMESPEED = 300.0;      // The speed for homing (steps per second)
const int delayBtwnInEx = 50;      //Changed value from 250 to 150// Delay between inspriration and expiration (milliseconds)
const int delayBtwnExIn = 20;       //Changed value from 250 to 20// Delay between expiration and inspiration (milliseconds)
const int cycleTmin = 5;            // Minimum allowable bpm
const int cycleTmax = 30;           // Maximum allowable bpm
const float tidalVmin = 50.00;      // Minimum compression percentage
const float tidalVmax = 100.00;     // Maximum compression percentage
const int ieRatiomin = 100;         // Minimum ratio of inspiration:expiration
const int ieRatiomax = 300;         // Maximum ratio of inspiration:expiration
const float maxStrokeFinger = 60.0; // Maximum allowable stroke of finger (degrees)
const float gearR = 1;             // Gear Ratio between stepper and finger, default is 16, including the gearbox is an additional *5
const float StepsPerRev = 800;     // Steps per revolution (setting on Applied Motion STR4 driver)
const bool useBlocking = false;     // Set to true to use blocking mode for inspiration/expiration
const long stepsAfterSW = 0.0;   // The number of steps after the expiration limit switch to wait to perform stop

// Initialize the LCD library with the number of interface pins
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

// Initialize AccelStepper, 1=Motor Driver
AccelStepper stepper = AccelStepper(1, STEP,DIR);

// Set-points
int cycleT = 0; //Cycle time duration set by user

// Timers
int cycleCount = 0; //Every 100 cycles, home

// Variables
float tidalVolume = 0;              // Tidal volume setpoint
int stepperStroke = 0;              // Commanded step count of stepper, calculated from tidalVolume
float ieRatio = 0;                  // inspiration:expiration set-point
bool estopLatch = false;            // Set E-stop to default off at bootup
float freq_in = 0;                  // Frequency for inspiration
float freq_ex = 0;                  // Frequency for expiration
int stepsCycle = 0;                 // number of motor steps per cycle 
long rotTime = 0.0;                 // Rotation time
float rotTime_in = 0.0;             // Rotation time of inspiration
float rotTime_ex = 0.0;             // Rotation time of expiration
long targetPos = 0.0;               // Target position calculated for inspiration and expiration
long cyclePos = 0.0;                // Position of cycle home (not used for position in this revision)
bool cyclePosSet = false;           // The cycle position has been set
long expTargetPos = 0.0;            // Target position after expiration limit switch
bool alarm = false;                 // Alarm status
float rotTime_in_corr = 0.0;
float rotTime_ex_corr = 0.0;


//Cycle watchdog
bool activeCycle = false;

void configureLCD() {
  lcd.begin(LCD_COLS, LCD_ROWS);                // initialize the lcd 
  lcd.home();                                   // go home
  lcd.clear();                                  // Clear LCD
}

void readInputs() {
    //Serial.println("Reading inputs"); // This will flood the Serial port; only use if debugging
    
    bool dip1 = digitalRead(DIP_SW1);
    bool dip2 = digitalRead(DIP_SW2);

    //***********Serial Debugging must be removed for the dip switches to work, the serial ports are on the
    //*********** same pins as the DIP switches
    if (dip1 && dip2) {             // For future use to add Tidal calculation manipulated by dip switches
      // DIP Switch 1 OFF and DIP Switch 2 OFF
      
    } else if (!dip1 && dip2) {
      // DIP Switch 1 ON and DIP Switch 2 OFF
      
    } else if (dip1 && !dip2) {
      // DIP Switch 1 OFF and DIP Switch 2 ON
      
    } else {
      // DIP Switch 1 ON and DIP Switch 2 ON
      
    }

    // replaced map() function with equations below. The Map() function only dealt with integers and the fractions were truncated. These equations are rounded before being converted to an integer to deal with noisy analog inputs
    cycleT = round(((float(abs(1023-analogReadAvg(BPM)))/1000.0) * float(cycleTmax - cycleTmin)) + float(cycleTmin)); //Reads the analog pot value and interpolates to the target range defined by the editable variables above. This can be bpm
    cycleT = constrain(cycleT, cycleTmin, cycleTmax);
    ieRatio = round(((float(abs(1023-analogReadAvg(Ratio)))/1000.0) * float(ieRatiomax - ieRatiomin)) + float(ieRatiomin));
    ieRatio = constrain(ieRatio, ieRatiomin, ieRatiomax) / 100.0;
    tidalVolume = round(((float(abs(1023-analogReadAvg(VolumeT)))/1000.0) * (tidalVmax - tidalVmin)) + tidalVmin);
    tidalVolume = constrain(tidalVolume, tidalVmin, tidalVmax) / 100.0;
    
    tidalVolume = constrain(tidalVolume, 0.0, 1.0);
      
    printSetpoints();
    stepsCycle = (tidalVolume * maxStrokeFinger * gearR ) * (StepsPerRev / 360.0); // motor steps per cycle 
    //Remap tidal volume to new floor ********Does this still need to be remapped?
    //tidalVolume = map(600,0,1000,70, tidalVmax)/100.00; //Remaps "50%" position of compression

    //Calculate cycle time parameters
    rotTime = (1000.0 * 60.0) / cycleT; // tone(pin, frequency, duration), where duration is in milliseconds 
    //rotTime_in = rotTime / (1 + ieRatio); Alexs fudge...replaced this line with one below, to correct for IE ratio error
    rotTime_in = rotTime / (1 + ((ieRatio)+(0.1*ieRatio*ieRatio)) - 0.7);
    rotTime_ex = rotTime - rotTime_in - delayBtwnInEx - delayBtwnExIn;    
    freq_in = stepsCycle * (1000.0 / rotTime_in); // frequency for compression stroke in Hz
    rotTime_in_corr = rotTime_in - (2*(freq_in/ACCEL));
    freq_ex = stepsCycle * (1000.0 / rotTime_ex); // frequency for compression stroke in Hz
    rotTime_ex_corr = rotTime_ex - (2*(freq_ex/ACCEL));
    freq_in = stepsCycle * (1000.0 / rotTime_in_corr);
    freq_in *= 1.09;
    freq_ex = stepsCycle * (1000.0 / rotTime_ex_corr); // frequency for compression stroke in Hz
    freq_ex *= 1.09;
}

// Averaging of analog input value
int analogReadAvg(int pin) {  //Stolen from UofT
  int analogSum = 0;
  for (int i = 0; i < 5; i++) {
    analogSum += analogRead(pin);
    delay(5);
  }
  return analogSum / 5;
}


void estop() { //Not implemented in this revision LCD cursor positions need to be adjusted to 2 columns with new LCD
    //digitalWrite(buzzer,HIGH);
    noTone(STEP);
    if (estopLatch == 0) {
      estopLatch = 1;
      lcd.clear();
      lcd.setCursor(3,1);
      lcd.print("ERROR DETECTED");
      lcd.setCursor(3,2);
      lcd.print("PLEASE REBOOT");
    }
}

// Debounces switchU, looks for 10 consecutive switch active readings
bool debounceswitchU()  
{
  static int isSwitchUActive = 0;               

  if (digitalRead(switchU) == 1) {
    isSwitchUActive++;
  } else {
    isSwitchUActive = 0;  
  }

  if (isSwitchUActive >= 25) {
    return 1;
  } else {
    return 0;
  }    
}

// Debounces switchH, looks for 10 consecutive switch active readings
bool debounceswitchH() 
{
  static int isSwitchHActive = 0;

  if (digitalRead(switchH) == 1) {
    isSwitchHActive++;
  } else {
    isSwitchHActive = 0;  
  }

  if (isSwitchHActive >= 25) {
    return 1;
  } else {
    return 0;
  }    
}

// Function to print setpoints to LCD
void printSetpoints() {                            
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BPM Rate:");
    lcd.setCursor(10,0);
    lcd.print(cycleT);
    lcd.setCursor(16,0);
    lcd.print("P 1");
    lcd.setCursor(0,1);
    lcd.print("I/E Ratio:");
    lcd.setCursor(10,1);
    lcd.print(ieRatio, 1);
    lcd.setCursor(0,2);
    lcd.print("Tidal Volume %:");
    lcd.setCursor(16,2);
    lcd.print(int(tidalVolume*100));
    lcd.setCursor(0,3);
    lcd.print("SNPIT&RC Ventilator");
  
}

// Homing function to home microswitch
void homeStepper() {
  cyclePosSet=false;                                           // Clear the cycle position set when homing
  alarm = false;                                               // Reset alarm
  
  Serial.println("Homing Stepper");
  lcd.clear();
  lcd.home();
  lcd.print("Homing Stepper");

  // If already at the home postion move off the microswitch a half a revolution and then home
  if (digitalRead(switchH)){
    stepper.setMaxSpeed(HOMESPEED);
    stepper.runToNewPosition(stepper.currentPosition() + (StepsPerRev/2));
    //Serial.println("moving off switch");
  }
  
  //Home stepper to home microswitch
  stepper.moveTo(stepper.currentPosition() - 1000000000);      // set a value to run continuously
  stepper.setMaxSpeed(HOMESPEED);                              // set speed of move to the homing speed
  while (!debounceswitchH()) {                                 // run the stepper while there is no home switch active
    stepper.run();
    //Serial.println("moving home");  
  }
  stepper.stop();                                              //home switch is active, controlled stop
  stepper.runToPosition();                                     //position calculated for controlled stop
  stepper.setCurrentPosition(0);                               //zero postion
}

// Move to the cycle home postion, based on switchU micro switch
void moveCyclePos() {
  Serial.println("Moving to Cycle Posistion");
  lcd.clear();     
  lcd.home();
  lcd.print("Moving to");
  lcd.setCursor(0,1);
  lcd.print("Cycle Position");
  
  stepper.moveTo(stepper.currentPosition() + 1000000000);      // set a value to run continuously
  stepper.setMaxSpeed(HOMESPEED);                              // set speed of move to the homing spee
  
  while (!debounceswitchU()) {                                 // run the stepper while there is switchU is not active
    stepper.run();
  }
  
  stepper.stop();                                              // switchU is active, controlled stop
  stepper.runToPosition();                                     // position calculated for controlled stop
  cyclePos = stepper.currentPosition();                        // set cycle position
  cyclePosSet = true;                                          
  delay(1000);                                                 // added delay before operation of ventilator
}

void setup() {
    Serial.begin(9600);                                        // Start serial communication for debugging
    configureLCD();                                            // Clears LCD
    Serial.println("Starting up...");
    lcd.print("Starting up...");

    // Configure inputs
    pinMode(switchU, INPUT_PULLUP);
    pinMode(switchH, INPUT_PULLUP);
    pinMode(startSwitch, INPUT_PULLUP);
    pinMode(DIP_SW1, INPUT_PULLUP);
    pinMode(DIP_SW2, INPUT_PULLUP);
    pinMode(buzzer, OUTPUT);
    
    stepper.setPinsInverted(true,true,true);                 // Direction pin inverted so that compression is positive direction
    stepper.setMaxSpeed(MAXSPEED);                             // Sets maxspeed
    stepper.setAcceleration(ACCEL);                            // Sets acceleration
    
    lcd.clear();
    delay(50);
    homeStepper();                                             // Inital homing of stepper motor
    Serial.println("Stepper homed");
    lcd.clear();
    lcd.home();
    lcd.print("Stepper homed");
    
    delay(1000);                                               // Delay added so user can read on LCD that the Stepper is homed
    
    readInputs();                                              // Read inputs to fill uncalculated variables
}

void loop() {
    
    if (!digitalRead(startSwitch) && !alarm) {          
        if (activeCycle == false) {

          // If the cycle postion has not been set after homing, move to cycle position
          if (!cyclePosSet) {
            moveCyclePos();
          }
          activeCycle = true;    
        }
        
        printSetpoints();

        Serial.println("inspiration");
        targetPos = stepper.currentPosition() - stepsCycle;     // Set target position for inspiration based on current position

        // Option to use blocking, set in constants
        if (useBlocking) {
          stepper.setMaxSpeed(freq_in);                         // Set speed for move
          stepper.runToNewPosition(targetPos);                  // Run to target position
        } else {
          stepper.moveTo(targetPos);                            // Set position to move to
          stepper.setMaxSpeed(freq_in);                         // Set speed for move

          // Run until target position reached
          while(stepper.currentPosition() > targetPos) {   //Add AND statement to while to include position switch if needed
            stepper.run();
          }    
          
          stepper.stop();                                       // Begin controlled stop
          stepper.runToPosition();                              // Run to calculated stop position
                  
          if (debounceswitchU()){                               // If the expiration limit switch is still on at the end of inspiration alarm
            alarm = true;
          }
        }

                // Reading and averaging the analog inputs takes up 75ms, 5 readings with a delay of 5ms times 3 inputs
        // The delay between expriation and inspiration with be adjusted for the analog input read

        if (delayBtwnInEx <= 75) {
          delay(1);                                             // Delay between expiration and inspiration is less than the analog read time
        } else {
          delay(delayBtwnInEx - 75);                            // Delay between expiration and inspiration
        }

        readInputs();                                           // Read the inputs every cycle
        //delay(delayBtwnInEx);                                   // Delay bewteen inspiration and expiration

        Serial.println("expiration");
        targetPos = stepper.currentPosition() + 5000000;     // Set target position for expiration based on current position
        
        if (useBlocking && !alarm) {
          stepper.setMaxSpeed(freq_ex);                         // Set speed for move
          stepper.runToNewPosition(targetPos);                  // Run to target position
        } else if (!useBlocking && !alarm) {
          stepper.moveTo(targetPos);                            // Set position to move to
          stepper.setMaxSpeed(freq_ex);                         // Set speed for move

          // Run until target position reached
          while(!debounceswitchU() && !debounceswitchH()) {     //Changed to based off limit switch
            stepper.run();        
          }

          if (debounceswitchH()){                               
            alarm = true;                                          // Alarm if switch missed and returns to home on expiration
          } else {
            expTargetPos = stepper.currentPosition() + stepsAfterSW;   // Sets new taget position for steps after limit switch to perform stop
            stepper.moveTo(expTargetPos);                              // Set position to move to
            stepper.setMaxSpeed(freq_ex);
          
            while(stepper.currentPosition() < expTargetPos) {        
              stepper.run(); 
            }
          }
          
          stepper.stop();                                       // Begin controlled stop
          stepper.runToPosition();                              // Run to calculated stop position
        }

        delay(delayBtwnExIn);
          
      }
      if (digitalRead(startSwitch)) {                           // No active cycle and confirm button pushed
          if (alarm){
            digitalWrite(buzzer, LOW);
            alarm = 0;
          }
          
          if (activeCycle == true ) {
            homeStepper();                                      // Runs once when transitioning from active to inactive, move to standby position
            activeCycle = false;
          }
          readInputs();
          delay(500);
      }

      if (alarm) {
        digitalWrite(buzzer, HIGH);
        if (!digitalRead(startSwitch)){
          delay(1000);
        }
        digitalWrite(buzzer, LOW);
        if (!digitalRead(startSwitch)){
          delay(1000);
        }
      }
}
