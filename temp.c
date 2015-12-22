#include <avr/io.h>

#include "adc.h"
#include "util.h"

// global GCU variables
int GCU_version;
int GCU_fill;
int n_temp;

// degrees C
#define COLDTEMP (25)

// uV / deg C
#define SENSITIVITY (41ul) // (41ul) for type K, (43ul) for type T

// voltage gain of differential amp stage
#define GAIN (100ul)

// #define COMPENSATE(x) ((((x) * (ADC_REF_V / (GAIN * SENSITIVITY))) / ADC_MAXCOUNT) + COLDTEMP)
#define COMPENSATE(x) ((((x) * (3981000l / (100ul * 41ul))) / 1024l) + 25)

unsigned long Temp_Data[16];
// unsigned int Temp_Corrections[16] = { 10000,10000,10000,10000,10000,10000,10000,10000,10023,9975,9987,10057,10010,10112,9932,9906};
int Temp_Available[16];

void Temp_Init() {
	int i;
	for (i=0; i<16; i++) {
		Temp_Data[i] = 0;
		Temp_Available[0] = 0;
	}
	if (GCU_fill == LITEFILL) {
		n_temp = 4;
		Temp_Available[0] = 1;
		Temp_Available[1] = 1;
		Temp_Available[2] = 1;
		Temp_Available[3] = 1;
	}
    if (GCU_fill == HALFFILL) {
		n_temp = 6;
		Temp_Available[0] = 1;
		Temp_Available[1] = 1;
		Temp_Available[2] = 1;
		Temp_Available[3] = 1;
		Temp_Available[4] = 1;
		Temp_Available[5] = 1;
	}
	if (GCU_fill == FULLFILL) {
		n_temp = 16;
	    Temp_Available[0] = 1;
		Temp_Available[1] = 1;
		Temp_Available[2] = 1;
		Temp_Available[3] = 1;
		Temp_Available[4] = 1;
		Temp_Available[5] = 1;
		Temp_Available[6] = 1;
		Temp_Available[7] = 1;
		Temp_Available[8] = 1;
		Temp_Available[9] = 1;
		Temp_Available[10] = 1;
		Temp_Available[11] = 1;
		Temp_Available[12] = 1;
		Temp_Available[13] = 1;
		Temp_Available[14] = 1;
		Temp_Available[15] = 1;
	}
}


void Temp_Reset() {
}


void Temp_ReadAll() {
	int bank, chan, i;
        unsigned long adder;

	for (bank=0; bank<4; bank++) {
		ADC_SETBANK(bank)
		for (chan=0; chan<4; chan++) {
                        adder = 0;
                        for (i=0; i<10; i++) {
                                 // adder += COMPENSATE(ADC_ReadChanSync(8 + chan));
                                 adder += ADC_ReadChanSync(8 + chan); // Temp_Data[4*chan + bank] += COMPENSATE(ADC_ReadChanSync(8 + chan));
                        }
                        //Temp_Data[4*chan + bank] = (adder * Temp_Corrections[4*chan + bank]) / 10000;
                        //Temp_Data[4*chan + bank] = adder;
                        Temp_Data[4*chan + bank] = adder;
		}
	}
}


#undef COLDTEMP
#undef SENSITIVITY
#undef GAIN
#undef COMPENSATE
