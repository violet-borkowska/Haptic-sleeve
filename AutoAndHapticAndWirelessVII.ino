#include <Wire.h>                 //Current Sensor  Initialisation
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#include <SPI.h>                  //Transmitter/Reciever Initialisation
#include <nRF24L01.h>
#include <RF24.h>

//PINS
int led = 2;                      //LED to show which state sleeve is in - Working state or Secure State
                                  //red LED
                                  //Working state (State 1) = Sleeve tigtens and loosens depending on grip pressure
                                  //Secure State (State 2) = Sleeve only tightens depending on grip pressure. When grip pressure is removed, remains at same tightness
int led2 = 3;                     //Untighten button
                                  //yellow LED
int motor1 = 4;                   //motor input 1,this shoudld be 3A (for this particular motor controller)
int motor2 = 5;                   //motor input 2, this should be 4A (for this particular motor controller)
int initcon = 6;                  //initial contact button
int adjt = 7;                     //adjustable tighten button
int adjut = 10;                   //adjustable untighten button
int fsr1 = A0;                    //Fingertip pressure sensor 1
int fsr2 = A1;                    //Fingertip pressure sensor 2

//VARIABLES
int state = 0;
int pos = 0;                      //position of sleeve motor
int ulim = 10;                    //Upper limit of tightness to comfort
int blim = 0;                     //Lower limit of tightness to comfort
int comfort = 5;                  //Comfort setting when sleeve is initially worn, higher comfort value means the sleeve tightens more
int q2 = 0;                       //Final readings of finger pressure sensor
int r2 = 0;
int intq1 = 0;                    //Reading of EMG 1
int extq1 = 0;                    //Reading of EMG 2
int avg1 = 0;                     //Calculate average
int avg2;                         //Calculate average
int counter = 0;                  //Counter
int lim = 15;                     //Limit of tightening for grip pressure
int i = 0;                        //Counter for FOR LOOP
int timer = 50;                   //Adjusts how much data is sent to reciever, smaller number means less data and reduced delay
int fsrv1 = 0;                    //Voltage of finger pressure sensors
int fsrv2 = 0;
int pulse = 0;
int k = 50;                       //Value to adjust delay - Bigger k means longer delays, Smaller k means smaller delays

long buttonTimer = 0;
long longPressTime = 250;

boolean buttonActive = false;
boolean longPressActive = false;

float current_mA = 0;             //Multiply current value by 10 to get mA


//WIRELESS
//VCC  = 3.3V;
//GND = GND;
const int pinCSN = 8;
const int pinCE = 9;
//MOSI = 11;
//SCK = 13;
//IRQ =
//MISO = 12;

int Array[4];

RF24 wirelessSPI(pinCE, pinCSN);
const uint64_t pAddress = 0xB00B1E5000LL;

void setup() {

  Serial.begin(9600);               //Current Sensor Initialisation
  uint32_t currentFrequency;
  ina219.begin();

  pinMode(initcon, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(led2, OUTPUT);
  pinMode(adjt, INPUT);
  pinMode(adjut, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);

  pinMode(fsr1, INPUT);
  pinMode(fsr2, INPUT);

  wirelessSPI.begin();
  wirelessSPI.setAutoAck(1);
  wirelessSPI.enableAckPayload();
  wirelessSPI.setRetries(5, 15);
  wirelessSPI.openWritingPipe(pAddress);
  wirelessSPI.stopListening();
  wirelessSPI.printDetails();
}

void motormotion(int m1, int m2, int val) {                       //Loop to turn motor
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  datasend(val);
  digitalWrite(m1, LOW);                                          //Stop the motors after sleeve is tightened
  digitalWrite(m2, LOW);
}

void untighten() {                                                //Loop to untighten the sleeve completely
  digitalWrite(motor2, HIGH);
  digitalWrite(motor1, LOW);

  while (pos > blim) {                                          //Whilst the motor position is greater than 0, untighten arm
    datasend(timer);
    pos--;                                                      //Position of motor decreased
  }

  digitalWrite(motor2, LOW);                                    //Switch motor off
  digitalWrite(motor1, LOW);
  state = 0;
  pos = 0;
  counter = 0;
  digitalWrite(led, LOW);
  datasend(timer * 3);
}

void motorpulse() {
  digitalWrite(motor1, HIGH);                                   //Sends a pulse to the user, so they know initial contact has been made
  digitalWrite(motor2, LOW);
  datasend(timer);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  datasend(timer / 2);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH);
  datasend(timer);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
}

int calcdel(int x) {

  int diff = x - avg1;
  counter = counter + diff;
  int del = abs(diff * k);
  return del;
}

int datacollect1() {
  fsrv1 = analogRead(fsr1);                                   //Reads FSR1 on the fingertip as analog (between 0 and 1023)
  q2 = (fsrv1 * (5 / 1023.0));                                //q2 now outputs a voltage value
  return q2;
}

int datacollect2() {
  fsrv2 = analogRead(fsr2);                                   //Reads FSR2 on the fingertip as analog (between 0 and 1023)
  r2 = (fsrv2 * (5 / 1023.0));                                //r2 now outputs a voltage value
  return r2;
}

void datasend(int j) {
  for (int i = 0; i < j; i++) {

    current_mA = (ina219.getCurrent_mA());                        //Begin showing the data. Data collected are: Motor current, and voltage from two FSR
    fsrv1 = analogRead(fsr1);                                     //Read voltage from FSR on fignertips
    fsrv2 = analogRead(fsr2);

    Array[1] = (current_mA / 10);                                 //Send 3 pieces of data to receiver
    Array[2] = ((fsrv1 * 5) / 1023);
    Array[3] = ((fsrv2 * 5) / 1023);

    if (!wirelessSPI.write( &Array, sizeof(Array))) {
    }
    else {
    }

  }
}

void loop() {

  //ARM IS NOT WORN

  while (state == 0) {
    datasend(1);
    int contact = digitalRead(initcon);                           //Reads the button inside arm to begin tightening sleeve

    if (contact == HIGH) {                                        //TIGHTENING ARM
      digitalWrite(motor1, HIGH);                                 //If button is HIGH, turn on motor for tightening to begin
      digitalWrite(motor2, LOW);
 
      while (pos < comfort) {
        datasend(timer);
        pos++;                                                   //Position of the motor has increased

        if (pos == comfort) {                                    //Turn motor off when comfort level is reached
          digitalWrite(motor1, LOW);
          digitalWrite(motor2, LOW);
          pos = comfort;
          state = 1;                                             //Go to other state, where arm is worn
        }
      }
    }
  }

  //--------------------------------------------------------------------------------------------------------------------------------
  //ARM WORN AND NEEDS REMOVING

  while (state == 1) {
    datasend(1);

    if (digitalRead(adjt) == HIGH) {                  //Held down adjt button would make it go to State 3 - adjustable tightness loop

      if (buttonActive == false) {
        buttonActive = true;
        buttonTimer = millis();
      }

      if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
        longPressActive = true;
        state = 3;
        digitalWrite(led2, HIGH);
        datasend(10);
      }
    }

    else {

      if (buttonActive == true) {

        if (longPressActive == true) {
          longPressActive = false;
        }

        else {
//          state = 2;
//          digitalWrite(led, HIGH);
        }
        buttonActive = false;
      }
    }

    int adjutoutput = analogRead(adjut) * 5 / 1023;                           //Hold down adjut button would untighten sleeve completely
    if (adjutoutput > 1) {
      datasend(timer / 3);
      int adjutoutput = analogRead(adjut) * 5 / 1023;
      if (adjutoutput > 1) {
        untighten();
      }
    }

    //ARM WORN AND USE GRIPPER IN WORKING STATE

    q2 = datacollect1();                                                 //Reads data from both FSRs
    r2 = datacollect2();

    if (((r2 > 1) & (q2 < 1)) | ((r2 < 1) & (q2 > 1))) {                //If only one FSR reads HIGH
      motorpulse();                                                     //Sends a pulse to user through arm (The sleeve will tighten and then untighten) 
                                                                        //showing that the user has made contact with an object
      pulse = 1;                                                        //User has made contact with an object, and how will test how strong the grip is

      while (pulse == 1) {
        q2 = datacollect1();                                            //Read data from both FSRs
        r2 = datacollect2();

        if ((r2 > 1) & (q2 > 1)) {                                      //Both FSRs read HIGH and sleeve will tighten/loosen depending on voltage outputs
          q2 = datacollect1();
          r2 = datacollect2();

          int avg2 = (r2 + q2) / 2;                                     //If HIGH (Both FSR1 and FSR2 is HIGH), then the sleeve tightens proportional to the
          if (avg2 > avg1 & counter < lim) {                            //Pressure applied to FSRs increasing, so sleeve will tighten
            int del = calcdel(avg2);
            motormotion(4, 5, del);
            avg1 = avg2;
          }

          if (avg2 < avg1 & counter > blim) {                           //Pressure applied to FSR decreases, sleeve will loosen
            int del = calcdel(avg2);
            motormotion(5, 4, del);
            avg1 = avg2;
          }
        }

        q2 = datacollect1();
        r2 = datacollect2();

        if ((r2 < 1) & (q2 < 1)) {                                     //If both FSRs are reading low, will untighten to comfort level

          if (counter > blim) {
            int del = calcdel(0);
            motormotion(5, 4, del);
            avg1 = 0;
            counter = 0;
          }
          pulse = 0;
          r2 = 0;
          q2 = 0;
          counter = 0;
          
        }
      }
    }
  }

    //------------------------------------------------------------------------------------------------------------------------------------------
//    while (state == 2) {
//
//          int adjutoutput = analogRead(adjut) * 5 / 1023;
//    if (adjutoutput > 1) {
//      datasend(timer / 3);
//      int adjutoutput = analogRead(adjut) * 5 / 1023;
//      if (adjutoutput > 1) {
//        untighten();
//      }
//    }
//
//      int adjtoutput = digitalRead(adjt);
//      if (adjtoutput == HIGH) {
//        state = 1;
//        digitalWrite(led, LOW);
//        datasend(10);
//      }
//
//      q2 = datacollect1();
//      r2 = datacollect2();
//
//      if (((r2 > 1) & (q2 < 1)) | ((r2 < 1) & (q2 > 1))) {                //If only one FSR reads HIGH
//        motorpulse();
//        pulse = 1;
//
//        while (pulse == 1) {
//          q2 = datacollect1();
//          r2 = datacollect2();
//
//          if ((r2 > 1) & (q2 > 1)) {                                      //Both FSRs read HIGH and sleeve will tighten/loosen depending on voltage outputs
//            q2 = datacollect1();
//            r2 = datacollect2();
//
//            int avg2 = (r2 + q2) / 2;                                     //If HIGH (Both FSR1 and FSR2 is HIGH), then the sleeve tightens proportional to the
//            if (avg2 > avg1 & counter < lim) {                            //Pressure applied to FSRs increasing, so sleeve will tighten
//              int del = calcdel(avg2);
//              motormotion(4, 5, del);
//              avg1 = avg2;
//            }
//          }
//
//          if (avg1 > 0) {
//
//            int adjutoutput = analogRead(adjut) * 5 / 1023;
//            if (adjutoutput > 3) {
//
//              if (counter > blim) {
//                int del = calcdel(0);
//                motormotion(5, 4, del);
//                avg1 = 0;
//                counter = 0;
//                pulse = 0;
//              }
//            }
//          }
//
//        if ((r2 < 1) & (q2 < 1)) {
//          pulse = 0;
//          r2 = 0;
//          q2 = 0;
//          
//        }
//          
//        }
//      }
//    }

    while (state == 3) {                                                            //ADJUST TIGHTNESS LOOP - If the sleeve needs to be adjusted from comfort because 
                                                                                    //it doesnt fit well (needs to be slightly tighter/looser), it will then remember
                                                                                    //this tightness setting when worn again

      if (digitalRead(adjt) == HIGH) {                                              //Holding down adjt button would make it go back to State 1

        if (buttonActive == false) {
          buttonActive = true;
          buttonTimer = millis();
        }

        if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
          longPressActive = true;
          state = 1;
          digitalWrite(led2, LOW);
        }
      }

      else {

        if (buttonActive == true) {                                                 //Pressing down adjt button would tighten the sleeve incrementally

          if (longPressActive == true) {
            longPressActive = false;
          }

          else {
            motormotion(4, 5, timer);
            comfort = comfort + 1;
            pos = pos + 1;
          }
          buttonActive = false;
        }
      }

      int adjutoutput = analogRead(adjut) * 5 / 1023;                               //Pressing down adjut button would loosen the sleeve incrementally
      datasend(1);
      adjutoutput = analogRead(adjut) * 5 / 1023;

      if ((adjutoutput > 3) && (counter == 0) && (pos > 1)) {
        motormotion(5, 4, timer);
        comfort = comfort - 1;
        pos = pos - 1;
      }

    }
  }
