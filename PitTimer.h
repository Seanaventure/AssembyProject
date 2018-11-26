#include "MKL46Z4.h"

#define PIT_ENABLE_TIMER 			0x1 //0b01
#define PIT_INTERUPT_ENABLE		0x2 //0b10
#define	PIT_TIMER_ENABLE			0x1	//0b01
#define PIT_TE_TIE_ENABLE			0x3 //0b11
#define PIT_MCR_FRZ						0x1 //0b01
#define PIT_MCR_EN_FRZ  (PIT_MCR_FRZ_MASK)
#define PIT_TCTRL_CH_IE  \
                       (PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK)
#define PIT_IPR_REGISTER      (PIT_IRQ_NUMBER >> 2)
#define NVIC_IPR_PIT_MASK     (3 << (((PIT_IRQ_NUMBER & 3) << 3) + 6))
#define PIT_IRQ_NUMBER 				22
#define NVIC_ICPR_PIT_MASK 		(1 << PIT_IRQ_NUMBER)
#define PIT_IRQ_PRIORITY 			0
#define NVIC_ISER_PIT_MASK    (1 << PIT_IRQ_NUMBER)

void initPIT(uint8_t);
void wait(uint8_t, int);
void setTimer(uint8_t, int);
int timerExpired(uint8_t);
