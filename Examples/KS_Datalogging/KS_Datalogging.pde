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
// Datalogging variables
unsigned int samplePeriod = 1000; // length of time between samples (call to loop())
long unsigned int nextTime = 0;
boolean augerOn;

void setup() {
  // timer initialization
  nextTime = millis() + samplePeriod;
  DoHeartBeatLED();
  delay(1);
  //Specify GCU: (version,fill,psequence)
  //version = V2 or V3 / 2.0 or 3.0
  //fill = HALFFILL or FULLFILL / half or full
  //psequence: for V3.01 boards, check sequence of P sensor part numbers (e.g. MXP7007 or MXP7002, and use last digit). V3.02 boards use P777722:
  // P777222, P222777, P777722
  GCU_Setup(V3,FULLFILL,P777722);
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
  if (GCU_version == V2 & GCU_fill == HALFFILL) {
    Serial.println("Time,T0,T1,T2,T3,T4,T5,P0,P2,ANA0,ANA1,ANA2,ANA3");
  } else if (GCU_version == V2 & GCU_fill == FULLFILL)  {
    Serial.println("Time,T0,T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T14,T15,P0,P1,P2,P3,ANA0,ANA1,ANA2,ANA3");
  } else if (GCU_version == V3 & GCU_fill == HALFFILL)  {
    Serial.println("Time,T0,T1,T2,T3,T4,T5,P0,P3,ANA0,ANA1,ANA2,ANA3");
  } else if (GCU_version == V3 & GCU_fill == FULLFILL)  {
    Serial.println("Time,T0,T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T14,T15,P0,P1,P2,P3,P4,P5,ANA0,ANA1,ANA2,ANA3");
  }
}

void loop() {
  int key;
  // first, read all KS's sensors
  Temp_ReadAll();  // reads into array Temp_Data[], in deg C
  Press_ReadAll(); // reads into array Press_Data[], in hPa
  Timer_ReadAll(); // reads pulse timer into Timer_Data, in RPM ??? XXX

// INSERT USER CONTROL CODE HERE
  if (millis() >= nextTime) {
    nextTime += samplePeriod;
    DoDatalogging();
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

void DoDatalogging() {
    // Serial output for datalogging
    Serial.print(millis()/1000); // time since reset in seconds
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
    if (GCU_fill == 2) {
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
    }
    for (int i= 0; i<6; i++) {
      if (Press_Available[i]) {
        Serial.print(Press_Data[i]);
        Serial.print(", ");
      }
    }
    Serial.print(ADC_ReadChanSync(0));
    Serial.print(", ");  
    Serial.print(ADC_ReadChanSync(1));
    Serial.print(", ");
    Serial.print(ADC_ReadChanSync(2));
    Serial.print(", ");
    Serial.print(ADC_ReadChanSync(3));
    Serial.print(", ");
    if (augerOn) {
      Serial.print("augerOn");
    } else {
      Serial.print("augerOff");
    }
    Serial.print("\r\n"); // end of line
}

void DoHeartBeatLED() {
  DDRJ |= 0x80;
  PORTJ |= 0x80;
}

void DoBEKAuger() {
  int AveBiomassTemp;
  AveBiomassTemp = (Temp[1] + Temp[2])/2;
  if (AveBiomassTemp > 405) {
    augerOn = true;
  }
  if (AveBiomassTemp < 395 & augerOn == true) {
    augerOn = false;
  }
  if (augerOn) {
    digitalWrite(FET0,HIGH);
  } else {
    digitalWrite(FET0,LOW);
  }
}
