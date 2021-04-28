//*************************************************************
//  Author: Alistair McConnell
//  Date 05/04/21 
//  Code for the experiments for myoelectric prothesis arm control
//  There are there basic parts in the main loop of this code:
//  (1) Comfy setting: This is where the sleeve is tightened to the desired comfort level
//  (2) Untighten: This is where the sleeve is loosen so it can be removed
//  (3) Experimental: This is the main experimental code where the operator presses 
//     button 1 and a loop of the code is run, the loop send all the fsr (voltage & force),
//     EMG, and time data through the serial conection where we use Putty to record it to 
//     a separate log file. 
//*************************************************************


#include <Wire.h>                 //Wire library 
#include <Adafruit_INA219.h>      //Current Sensor  Initialisation
#include <SPI.h>                  //Transmitter/Reciever Initialisation
//#include <nRF24L01.h> 
//#include <RF24.h>
Adafruit_INA219 ina219;

//PINS
int led = 2;                      //Green LED    
int led2 = 3;                     //Red LED                                 
int pwm1 = 9;                     //PWM control 
int motor1 = 10;                  //motor input 1 (EN1)
int motor2 = 11;                  //motor input 2 (EN2)
int button1 = 6;                  //Bottom button
int button2 = 7;                  //Middle button
int button3 = 5;                  //Top Button
int fsr1 = A0;                    //Fingertip pressure sensor 1
int fsr2 = A1;                    //Fingertip pressure sensor 2
int EMG_C1 = A3;                  //EMG1 sensor 
int EMG_C2 = A4;                  //EMG2 sensor

//VARIABLES
int i = 0;                        //Variable used for intial motor check
int fsrv1;                        //Voltage of finger pressure sensors 1 (Max voltage when no force applied)(mV)
int fsrv2;                        //Voltage of finger pressure sensors 2 (mV)
int cal_fsr1;                     //Mapped voltage of finger pressure sensor 1
int cal_fsr2;                     //Mapped voltage of finger pressure sensor 2
int cal_fsr;                      //
int total_fsrv;                   //
int fsr1Voltage;                  //FSR1 voltage (V) 
int fsr2Voltage;                  //FSR2 voltage (V)
float fsr1Resistance;               //FSR1 Resistance (mOhms)
float fsr2Resistance;               //FSR2 Resistance (mOhms)
float fsr1Conductance;              //FSR1 Conductance 
float fsr2Conductance;              //FSR2 Conductance 
float fsr1Force;                    //FSR1 Force (N)
float fsr2Force;                    //FSR2 Force (N)
int EMGC1;                        //EMG1 data
int EMGC2;                        //EMG2 data
int cal_EMGC1;                    //Mapped value of EMG 1
int cal_EMGC2;                    //Mapped value of EMG 2
int comfy = 0;                    //Number of turn for sleeve to be comfy
int sleeve;                       //Number of turn the sleeve need to complete to return to comfy setting
float current_mA = 0;             //Multiply current value by 10 to get mA
const float R_DIV = 10000.0;      //Measured resistance of 10k resistor
int contact1;                     //Bottom button variable
int contact2;                     //Middle button variable
int contact3;                     //Top Button variable 
int cal_fsr_old;
int current_time_s;
unsigned long cycle_start_time;
unsigned long current_time;
int cycle_start_time_s;
int cycle = 0;
 
void setup() {

  Serial.begin(115200);           //Set the serial comunications speed
  ina219.begin();               //Begin the current sensor reading

  pinMode(led, OUTPUT);         //Set led pin as an output 
  pinMode(led2, OUTPUT);        //Set led2 pin as an output
  pinMode(button1, INPUT);      //Set button1 pin as an input
  pinMode(button2, INPUT);      //Set button2 pin as an input
  pinMode(button3, INPUT);      //Set button3 pin as an input
  pinMode(motor1, OUTPUT);      //Set motor1 pin as an output
  pinMode(motor2, OUTPUT);      //Set motor2 pin as an output
  pinMode(pwm1, OUTPUT) ;       //Set PWM pin as and output
  digitalWrite(motor1, LOW);    //Set motor1 default to LOW or off 
  digitalWrite(motor2, LOW);    //Set motor2 default to LOW or off 
  pinMode(fsr1, INPUT);         //Set fsr1 pin as an input
  pinMode(fsr2, INPUT);         //Set fsr2 pin as an input
  pinMode(EMG_C1, INPUT);       //Set fsr2 pin as an input
  pinMode(EMG_C2, INPUT);       //Set fsr2 pin as an input

Serial.println((String) "fsr1" + ";" + "fsr2" + ";" + "FSR Sum" + ";" + "FSR1 Force" + ";" + "FSR2 Force" + ";" + "EMG1" + ";" + "EMG2" + ";" + "Cycle" + ";" + "Time");
}

void loop() {

//-------------------------------------------Starting Point-------------------------------------------------------
// Set motor to zero,turn motor forward for 0.5sec, show current reading, turn back 0.5sec, turn motor off, blink each LED, show FSR readings,send data to excel/text file
// Wait for a button to be pressed to trigger a function

while (i < 1)
{
      digitalWrite(led, HIGH);     //     
      digitalWrite(led2, LOW);     //
      digitalWrite(motor2, HIGH);  //
      digitalWrite(motor1, LOW);   //
      analogWrite(pwm1, 255) ;     //
      delay (500);                 //
      digitalWrite(led, LOW);      //    
      digitalWrite(led2, HIGH);    //
      digitalWrite(motor2, LOW);   //
      digitalWrite(motor1, HIGH);  //
      analogWrite(pwm1, 255) ;     //
      delay (500);                 //
      digitalWrite(motor2, LOW);   //
      digitalWrite(motor1, LOW);   //
      digitalWrite(led, LOW);      //     
      digitalWrite(led2, LOW);     //
      i = 1;
}

int contact1 = digitalRead(button1);    //Read the state of the bottom button and store it in the variable contact1
int contact2 = digitalRead(button2);    //Read the state of the middle button and store it in the variable contact2
int contact3 = digitalRead(button3);    //Read the state of the top button and store it in the variable contact3
                     
delay (10);



// -------------------------------------------Program 1: Untighten-------------------------------------------
// (1) LEDs off
// (2) Motor reverses fully and turns off
// (3) Zero variables 
// (4) LEDs blink twice to show reset it complete 

if (contact1 == HIGH)                       //If statement to check if the bottom button is presssed (if pressed do x else do nothing)
    {                      
      digitalWrite(motor1, HIGH);           //Untighten the sleeve
      digitalWrite(motor2, LOW);                   
      analogWrite(pwm1, 255) ;              //Set PWM to max 
      digitalWrite(led, HIGH);              //Turn on the green LED
      delay (1000);
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(led, LOW);               //Turn off the green LED  
     // comfy--;                              //Number of time the motor has run for 1 second at full speed
    }

// -------------------------------------------Program 2: Comfort Set-------------------------------------------
// Once sleeve is attached to the human, press button (2) to set comfort position 
// (1) Press button (2)
// (2) Motor tightens for 0.5sec at half speed
// (3) Press button (2) 
// (4) Motor tightens for 0.5sec at half speed
// (5) Repeat till comfortably tight
// (6) Hold button down to exit function
// (7) Number of button presses is recorded as the comfort level

if (contact2 == HIGH)                       //If statement to check if the middle button is presssed (if pressed do x else do nothing)
    {                      
      digitalWrite(motor1, LOW);            //Tighten the sleeve
      digitalWrite(motor2, HIGH);                   
      analogWrite(pwm1, 255) ;              //Set PWM to max 
      digitalWrite(led2, HIGH);             //Turn on the red LED
      delay (1000);                         //Run the motor for 1 second
      digitalWrite(motor1, LOW);            //Turn off the motor
      digitalWrite(motor2, LOW);
      digitalWrite(led2, LOW);              //Turn off the red LED  
      comfy++;                              //Number of time the motor has run for 1 second at full speed   
    }

// -------------------------------------------Program 3: FSR Controlled Squeeze-------------------------------------------
// Press button (3), set sleeve to comfort position, tighten sleeve relative to FSR readings, return to comfort postions at 0 reading for FSR
// (1) Set sleave to comfort position
// (2) Read FSR readings
// (3) Map the FSR readings to 0 - 255
// (4) Sum the FSR readings 
// (5) Map sum of FSR readings to 0 - 255pwm
// (6) Calculate all related FSR values (resitance, force etc)
// (6) Read in EMG1 and EMG2 data
// (7) Print all readings to serial
// (8) Check if the FSR readings are more or less than previous reading
// (9) Make current FSR sum reading the old one 
// (10) Reverse the motor to loosen the sleave
// (11) Put sleave back to comfy setting

if (contact3 == HIGH)                           //If statement to check if the top button is presssed (if pressed do x else do nothing)
    {     
     delay (1000);
      digitalWrite(led, HIGH);     //Flash green LED
      delay (1000);
       digitalWrite(led, LOW);     //
       delay (1000);
        digitalWrite(led, HIGH);     //
        delay (1000);
         digitalWrite(led, LOW);     //
         delay (1000);
         digitalWrite(led, HIGH);     //
          delay (1000);
          digitalWrite(led, LOW);     //
         
     int average_fsr = 0;
     int average_fsr_old = 0;
     int cal_fsr_old = 0;
     int sleeve = 0;;
    
     digitalWrite(motor1, LOW);                             //Set motor 1 to LOW
     digitalWrite(motor2, LOW);                             //Set motor 2 to LOW
     
 //    Serial.println((String) "fsr1" + ";" + "fsr2" + ";" + "FSR Sum" + ";" + "FSR1 Force" + ";" + "FSR2 Force" + ";" + "EMG1" + ";" + "EMG2" + ";" + "Cycle" + ";" + "Time");
     cycle_start_time = millis();                           //Time (milli sec) since the start of the current cycle 
     cycle_start_time_s = (cycle_start_time/1000);          //Time (sec) since the start of the current cycle 
     current_time = millis();                               //Time (milli sec) since starting up arduino
     current_time_s = (current_time/1000);                  //Time (sec) since starting up arduino
                
     while (cycle_start_time - current_time <= 10000)    
            {            
                fsrv1 = analogRead(fsr1);                   //Read the state of the FSR1 sensor and store it in the variable fsrv1 (analog reading so you need to use analogread)
                cal_fsr1 = map(fsrv1, 0, 1023, 0, 255);     //Mapping the fsr1 reading to between 0 - 255 for easy PWM convertion                 
                fsrv2 = analogRead(fsr2);                   //Read the state of the FSR2 sensor and store it in the variable fsrv2 (analog reading so you need to use analogread)
                cal_fsr2 = map(fsrv2, 0, 1023, 0, 255);     //Mapping the fsr2 reading to between 0 - 255 for easy PWM convertion 
                total_fsrv = (fsrv1 + fsrv2);               //The sumation of the fsr readings
                cal_fsr = map(total_fsrv, 0, 1600, 0, 255); //This is the sum of the fsr readings mapped to what should be 2046 but may vary in reality
                               
                digitalWrite(motor1, LOW);                 //Set the motor to "on"
                digitalWrite(motor2, HIGH);
                
              if (cal_fsr > cal_fsr_old)                 //If last average-fsr was less than new average fsr drive forward motor
                  {
                    analogWrite(pwm1, cal_fsr1);              //Tighten at the speed constant with the force applied to the object  
                    delay (100);
                  }
              else
              {
                digitalWrite(motor1, LOW);                 //Set the motor to "on"
                digitalWrite(motor2, LOW);
                delay (100);
              }
                  
                fsr1Voltage = map(fsrv1, 0, 1023, 0, 5000); // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
                fsr2Voltage = map(fsrv2, 0, 1023, 0, 5000); // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
                
                fsr1Resistance = 5000 - fsr1Voltage;                // fsrVoltage is in millivolts so 5V = 5000mV 5000 - 2500 = 2500
                fsr1Resistance *= 10000;                            // 10K resistor 25,000,000 2500*10000 = 25,000,000
                fsr1Resistance /= fsr1Voltage;                      // 25,000,000/2500 = 10,000

                fsr2Resistance = 5000 - fsr2Voltage;                // fsrVoltage is in millivolts so 5V = 5000mV
                fsr2Resistance *= 10000;                            // 10K resistor
                fsr2Resistance /= fsr2Voltage;
                 
                fsr1Conductance = 1000000;                          // we measure in micromhos so 
                fsr1Conductance /= fsr1Resistance;                  // 1000000/10000 = 100

                fsr2Conductance = 1000000;                          // we measure in micromhos so 
                fsr2Conductance /= fsr2Resistance;

                // Use the two FSR guide graphs to approximate the force from the first fsr
                if (fsr1Conductance <= 1000) {
                  fsr1Force = fsr1Conductance / 80;     
                } 
                else 
                {
                  fsr1Force = fsr1Conductance - 1000;
                  fsr1Force /= 30;      
                }

                // Use the two FSR guide graphs to approximate the force from the second fsr
                if (fsr2Conductance <= 1000) {
                  fsr2Force = fsr2Conductance / 80;     
                } 
                else 
                {
                  fsr2Force = fsr2Conductance - 1000;
                  fsr2Force /= 30;      
                }
                
                EMGC1 = analogRead(EMG_C1);                 //Read the state of the EMG1 sensor and store it in the variable EMGC1 (analog reading so you need to use analogread)
                EMGC2 = analogRead(EMG_C2);                 //Read the state of the EMG2 sensor and store it in the variable EMGC2 (analog reading so you need to use analogread)
                Serial.println((String) fsrv1 + ";" + fsrv2 + ";" + total_fsrv + ";" + fsr1Force + ";" + fsr2Force + ";" + EMGC1 + ";" + EMGC2 + ";" + cycle + ";" + cycle_start_time);                   
                 cal_fsr_old = cal_fsr;                     //Makes the new average FSR reading the old one for next iteration comparison
                 cycle_start_time = millis();
                
            }
            
            digitalWrite(motor1, HIGH);                      // Untighten the sleeve
            digitalWrite(motor2, LOW);
            analogWrite(pwm1, 255);
            delay (3000);                                   // A five second delay should allow for the motor to fully unwind the sleeve
            digitalWrite(motor1, LOW);                      // Untighten the sleeve
            digitalWrite(motor2, LOW);
            
//            while (sleeve <= comfy)                          // Loop to tighten the sleeve to the comfy setting
//                  {
//                    digitalWrite(motor1, LOW);
//                    digitalWrite(motor2, HIGH);
//                    analogWrite(pwm1, 255);
//                    delay (1000);
//                    sleeve++;
//                  }
            cycle++;                                         // Add 1 to the cycle number     
      }
      
//      digitalWrite(motor1, LOW);
//      digitalWrite(motor2, HIGH);
//      analogWrite(pwm1, 255);
//      delay (5000);                                          // A five second delay should allow for the motor to fully unwind the sleeve    
}                                                                    
                      
