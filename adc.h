#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANA0 54 //Arduino pin 54 = ANALOG 0
#define ANA1 55 //Arduino pin 55 = ANALOG 1
#define ANA2 56 //Arduino pin 56 = ANALOG 2
#define ANA3 57 //Arduino pin 57 = ANALOG 3

void ADC_Init();
void ADC_Reset();
uint16_t ADC_ReadChanSync(unsigned chan);

// 10 bits
#define ADC_MAXCOUNT (1024l)

// in microvolts
#define ADC_REF_V (5000000l)

// set MUXA / MUXB pins to correct bank
#define ADC_SETBANK(n) {PORTL = (PORTL & ~0xc0) | ((n) << 6);}


#ifdef __cplusplus
}
#endif
