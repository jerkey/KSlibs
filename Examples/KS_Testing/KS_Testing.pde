//#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PID_Beta6.h>
#include <adc.h>
#include <display.h>
#include <fet.h>
#include <keypad.h>
#include <pressure.h>
#include <servo.h>
#include <temp.h>
#include <timer.h>
#include <ui.h>
#include <util.h>
#include <avr/io.h>  

// KS pin out definitions
#define FET0 5     //Arduino pin 5 = FET 0      Timer:TCCR3B                       
#define FET1 2     //Arduino pin 2 = FET 1      Timer:TCCR3B                       
#define FET2 3     //Arduino pin 3 = FET 2      Timer:TCCR3B                       
#define FET3 6     //Arduino pin 6 = FET 3      Timer:TCCR4B                       
#define FET4 7     //Arduino pin 7 = FET 4      Timer:TCCR4B                       
#define FET5 8     //Arduino pin 8 = FET 5      Timer:TCCR4B                       
#define FET6 46    //Arduino pin 46 = FET 6     Timer:TCCR5B                       
#define FET7 45    //Arduino pin 45 = FET 7     Timer:TCCR5B                       
#define SERVO0 11  //Arduino pin 11 = SERVO 0   Timer:TCCR1B                       
#define SERVO1 12  //Arduino pin 12 = SERVO 1   Timer:TCCR1B                       
#define SERVO1 13  //Arduino pin 13 = SERVO 2   Timer:TCCR0B Freq = 2x            
#define ANA0 54    //Arduino pin 54 = ANALOG 0
#define ANA1 55    //Arduino pin 55 = ANALOG 1
#define ANA2 56    //Arduino pin 56 = ANALOG 2
#define ANA3 57    //Arduino pin 57 = ANALOG 3

//Sampling Variables
unsigned int samplePeriod = 1000; // length of time between samples (call to loop())
long unsigned int nextTime = 0;

//Experimental Design
int test_number = -2;
// test number	    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 
int hot_tests[] = { 3, 3, 2, 1, 0, 0, 1, 2, 3, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 1, 2, 3, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0,-99};
// test number      0  1  2  3  4  5 6
int cold_tests[] = {5, 4, 3, 2, 1, 0,-99};
// test pressure        0   1    2    3    4     5
//                      1/4 1    2    4    8     10 // inH2O
int test_pressures[] = {63, 249, 498, 996, 1993, 2491}; // in Pascals
int test_length = 300; //300 s = 5 min
int cold_test_length = 30;
int test_time = 0;
boolean cold_test = false;

//Flow Measurement
float CfA0_air =0.42123;
float CfA0_gas = 0.651105695; //0.651105695 (1.5" welded, 0.76"), 0.8105 (1.5" union, 0.75")
float sensor_offset_air = 0;
float sensor_offset_gas = 0;

//Calibration
int Press_Calib[4];
int Press[4]; //values corrected for sensor offset (calibration)

// Ejector Flow Variables
double flow_setpoint, flow_input, flow_output, pressure_setpoint;
PID flow_PID(&flow_input, &flow_output, &pressure_setpoint,2,1,5); //.3,10,0
float servo_pos = 0;

//Grate Turning Variables
boolean grate_on;
int GRATE_MOTOR_FWD = FET3;
int GRATE_MOTOR_REV = FET2;
int grate_interval = 6000; // count of ~10 ms loops, should move to clock to avoid dependencies
int grate_length = 16;
int grate_timer;
float reduction_resistance_sum;
float reduction_resistance;
float reduction_resistance_count;

//Flow Variables
double air_flow;
double gas_flow;

// PID tuning variables
double Setpoint, Output, Input;
unsigned long serialTime; //this will help us know when to talk with processing
boolean PIDtuning = false;

void setup() {
  // timer initialization
  nextTime = millis() + samplePeriod;
   
  // ejector initialization - may need adjusted for different setups
  flow_setpoint = 20;
  flow_output = 50;
  flow_PID.SetSampleTime(10);
  flow_PID.SetInputLimits(-5000,5000);
  flow_PID.SetOutputLimits(0,160);
  flow_PID.SetMode(AUTO); 

  //  
  DDRJ |= 0x80;      
  PORTJ |= 0x80;   
  PRR0 = 0x00;
  PRR1 = 0x00;  // turn on EVERYTHING
  delay(1);	

  Disp_Init();
  Kpd_Init();
  UI_Init();
  ADC_Init();
  Temp_Init();
  Press_Init();
  Fet_Init();
  Servo_Init();
  Timer_Init();

  Disp_Reset();
  Kpd_Reset();
  UI_Reset();
  ADC_Reset();
  Temp_Reset();
  Press_Reset();
  Fet_Reset();
  Servo_Reset();
  Timer_Reset();

  Serial.begin(115200);
  
  LoadPressureSensorCalibration();
}

void loop() {
  int key;
  // first, read all KS's sensors
  Temp_ReadAll();  // reads into array Temp_Data[], in deg C
  Press_ReadAll(); // reads into array Press_Data[], in hPa
  Timer_ReadAll(); // reads pulse timer into Timer_Data, in RPM ??? XXX
  for (int i = 0; i<4; i++) {
    Press[i] = Press_Data[i]-Press_Calib[i];
  }
// INSERT USER CONTROL CODE HERE
  
  air_flow = CalcFlowmeter(CfA0_air, -Press[3], 1.2, sensor_offset_air);
  gas_flow = CalcFlowmeter(CfA0_gas, -Press[2], 1.2, sensor_offset_gas);
  
  DoSerialIn(); // accept terminal control
  DoEjector();
  DoGrate();
  if (millis() >= nextTime) {
    nextTime += samplePeriod;
    DoExperiment(); // change variables for test
    
    reduction_resistance_sum += (Press[0]-Press[1])/gas_flow;
    reduction_resistance_count++;
    
    logdata(); // print data to terminal
  }
  
// END USER CONTROL CODE

  UI_DoScr();       // output the display screen data, 
                    // (default User Interface functions are in library KS/ui.c)
                    // XXX should be migrated out of library layer, up to sketch layer                      
  key = Kpd_GetKeyAsync();
                    // get key asynnchronous (doesn't wait for a keypress)
                    // returns -1 if no key

  UI_HandleKey(key);  // the other two thirds of the UI routines:
                      // given the key press (if any), then update the internal
                      //   User Interface data structures
                      // ALSO: Manipulate the various output data structures
                      //   based on the keypad input

  Fet_WriteAll();   // Write the FET output data to the PWM hardware
  Servo_WriteAll(); // Write the Futaba hobby servo data to the PWM hardware
        
  PORTJ ^= 0x80;    // toggle the heartbeat LED
  delay(10);
}

void logdata() {
    // Serial output for datalogging
    Serial.print(millis()/1000); // time since restart in seconds
    Serial.print(", ");
    Serial.print(Temp_Data[0]);
    Serial.print(", ");
    Serial.print(Temp_Data[1]);
    Serial.print(", ");
    Serial.print(Temp_Data[2]);
    Serial.print(", ");
    Serial.print(Temp_Data[3]);
    Serial.print(", ");
    Serial.print(Temp_Data[4]);
    Serial.print(", ");
    Serial.print(Temp_Data[5]);
    Serial.print(", ");
    Serial.print(Temp_Data[6]);
    Serial.print(", ");
    Serial.print(Temp_Data[7]);
    Serial.print(", ");
    Serial.print(Temp_Data[8]);
    Serial.print(", ");
    Serial.print(Temp_Data[9]);
    Serial.print(", ");
    Serial.print(Temp_Data[10]);
    Serial.print(", ");
    Serial.print(Temp_Data[11]);
    Serial.print(", ");
    Serial.print(Temp_Data[12]);
    Serial.print(", ");
    Serial.print(Temp_Data[13]);
    Serial.print(", ");
    Serial.print(Temp_Data[14]);
    Serial.print(", ");
    Serial.print(Temp_Data[15]);
    Serial.print(", ");
    Serial.print(Press[0]);
    Serial.print(", ");
    Serial.print(Press[1]);
    Serial.print(", ");
    Serial.print(Press[2]);
    Serial.print(", ");
    Serial.print(Press[3]);
    Serial.print(", ");
    if (grate_on) {
    Serial.print("on");
    } else {
    Serial.print("off");
    }
    Serial.print(", ");
    Serial.print(air_flow);
    Serial.print(", ");
    Serial.print(gas_flow);
    Serial.print(", ");
    Serial.print(pressure_setpoint);
    Serial.print(", ");
    Serial.print(flow_input);
    Serial.print(", ");
    Serial.print(flow_output);
    Serial.print(", ");
    //Serial.print(servo_pos);
    //Serial.print(", ");
    Serial.print(ADC_ReadChanSync(0));
    Serial.print(", ");  
    Serial.print(ADC_ReadChanSync(1));
    Serial.print(", ");
    Serial.print(ADC_ReadChanSync(2));
    Serial.print(", ");
    Serial.print((Press[0]-Press[1])/gas_flow); //resistance
    Serial.print(", ");
    Serial.print(test_number);
    Serial.print("\r\n"); // end of line
    grate_on = false;
}

void DoExperiment() {
  int length = 0;
  int test;
  if (cold_test) {
    length = cold_test_length;
  } else {
    length = test_length;
  }
  test_time++;
  if (test_time >= length && test_number >= 0) {
    test_number++;
    PrintExpTest();
    test_time = 0;
  }
  if (test_number == -2) { // low off
    pressure_setpoint = 50;
  }
  if (test_number == -1) { // ignition
    pressure_setpoint = 1000;
  }
  if (test_number>=0) {
    if (cold_test) {
      test = cold_tests[test_number];
    } else {
      test = hot_tests[test_number];
    }
    if (test==-99) {
      test_number = -2;
      cold_test = false;
    } else {
      pressure_setpoint = test_pressures[test];
    }
  }
}

void DoSerialIn() {
  int incomingByte = 0;
  // Serial input
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 112: //p
          CalibratePressureSensors();
          LoadPressureSensorCalibration();
          break;
      case 97: //a
          test_number--;
          PrintExpTest();
          test_time = 0;
          break;
      case 100: //d
          test_number++;
          PrintExpTest();
          test_time = 0;
          break;
      case 99: //c
        if (cold_test) {
          cold_test = false;
          test_number = -2;
          PrintExpTest();
        } else {
          cold_test = true;
          test_number= 0;
          PrintExpTest();
        }
        break;
    }
  }
}

void PrintExpTest() {
  Serial.print("#");
  Serial.print(millis()/1000);
  Serial.print("Test number now: ");
  Serial.print(test_number);
    if (cold_test) {
    Serial.print(" cold");
  } else {
    Serial.print(" hot");
  }
  Serial.println();
}

void DoEjector() {
  // do air in set point
  //flow_setpoint=map(analogRead(0),534,715,0,50); // read setpoint from potientometer
  
  //Uncomment for potentiometer based pressure setting
  //pressure_setpoint=map(analogRead(0),20,500,0,2000); // read setpoint from potientometer
  flow_input=-Press[1];
  flow_PID.Compute();
  PulseServo(SERVO0,flow_output); //this should move to a servo library, would use a FET and PWM to control a proportional solenoid
  //PulseServo(SERVO0,servo_pos); //for calibration
}

void DoGrate() { // control a grate shaking motor with relays configured for bi-directional control
  grate_timer++;
  if (grate_timer > (grate_interval-grate_length*2)) {
      grate_on = true;
      if ((grate_interval-grate_timer)>=grate_length) {
        analogWrite(GRATE_MOTOR_FWD, 255);
        analogWrite(GRATE_MOTOR_REV, 0);
      } else {
        analogWrite(GRATE_MOTOR_FWD, 0);
        analogWrite(GRATE_MOTOR_REV, 255);
      }
  } else {
    analogWrite(GRATE_MOTOR_FWD, 0);
    analogWrite(GRATE_MOTOR_REV, 0);
  }
  if (grate_timer >= grate_interval) {
    grate_timer = 0;
  }
}

//Flowmeter Equation
double CalcFlowmeter(double CfA0, double dP, double density, double offset) {
  return CfA0*sqrt(((dP-offset)*2)/density);
}

void PulseServo(int servo_pin,double angle) {
  // manually pulse servo - up to 2 ms delay will be be an issue w/ more then a few servos, may be ok w/ 3.
  digitalWrite(SERVO0,HIGH);
  delayMicroseconds(1000+angle*5.556);
  digitalWrite(SERVO0,LOW);
}

void CalibratePressureSensors() {
  int P_sum[4] = {0,0,0,0},P_ave;
  byte lowbyte,highbyte;
  Serial.println("Calibrating Pressure Sensors");
  for (int i=0; i<10; i++) {
    Press_ReadAll();
    for (int j=0; j<4; j++) {
      P_sum[j] += Press_Data[j];
    }
    delay(1);
  }
  //write to EEPROM
  for (int i=0; i<4; i++) {
    P_ave = float(P_sum[i])/10.0;
    lowbyte = ((P_ave >> 0) & 0xFF);
    highbyte = ((P_ave >> 8) & 0xFF);
    EEPROM.write(i*2, lowbyte);
    EEPROM.write(i*2+1, highbyte);
  }
}

void LoadPressureSensorCalibration() {
    int calib;
    byte lowbyte,highbyte;
    Serial.println("Loading Pressure Sensor Calibrations:");
    for (int i=0; i<4; i++) {
      byte lowByte = EEPROM.read(i*2);
      byte highByte = EEPROM.read(i*2 + 1);
      Press_Calib[i] = ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
      Serial.print("P");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(Press_Calib[i]);
      Serial.println();
    }
}
