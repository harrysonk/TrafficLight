// TableTrafficLight.c
// Runs on LM4F120/TM4C123
// Use a table implementation of a Moore finite state machine to operate
// a traffic light.
// Original by: Daniel Valvano - February 2, 2015
// Revision by: Gursel Serpen - June 18, 2017

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)
// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"

// Label input/output ports
#define LIGHT                   (*((volatile uint32_t *)0x400050FC))
#define SENSOR                  (*((volatile uint32_t *)0x4002401C))
#define GPIO_PORTF_OUT          (*((volatile uint32_t *)0x40025028))

// Define digital input/output ports
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C))
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))

struct State {
  uint32_t Out[2];      // 6-bit output
  uint32_t Time;     // 10 ms
  uint8_t Next[8];}; // depends on 3-bit input
typedef const struct State STyp;
	
#define goS   0
#define waitS 1
#define goW   2
#define waitW 3
#define stopWaitS 4
#define stopWaitW 5
#define stopSW 6
#define stopBlink1SW 7
#define stopBlink2SW 8
#define stopBlink3SW 9
#define stopBlink4SW 10
#define stopSolidSW 11

// Define FSM as global	
//The walk sensor will be PE2
//The last state added will run for 10 seconds (how long pedestrians have to cross), both lights are red in this case
//Walk light will stay on for 2 seconds, then dont walk light will blink for 2 seconds, and then dont walk light will
//Stay solid for 2 seconds
STyp FSM[12]={
 {{0x21,0x02}, 200,{goS,waitS,goS,waitS, stopWaitS, stopWaitS, stopWaitS, stopWaitS}},
 {{0x22, 0x02}, 50,{goW,goW,goW,goW, stopSW, stopSW, stopSW, stopSW}},
 {{0x0C, 0x02},200,{goW,goW,waitW,waitW, stopWaitW, stopWaitW, stopWaitW, stopWaitW}},
 {{0x14, 0x02}, 50,{goS,goS,goS,goS, stopSW, stopSW, stopSW, stopSW}},
 {{0x22, 0x02}, 50,{stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW}},
 {{0x14, 0x02}, 50,{stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW, stopSW}},
 {{0x24, 0x08},200,{stopBlink1SW,stopBlink1SW,stopBlink1SW,stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW, stopBlink1SW}},
 {{0x24,0x02},50,{stopBlink2SW,stopBlink2SW,stopBlink2SW,stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW, stopBlink2SW}},
 {{0x24,0x00},50,{stopBlink3SW,stopBlink3SW,stopBlink3SW,stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW, stopBlink3SW}},
 {{0x24,0x02},50,{stopBlink4SW,stopBlink4SW,stopBlink4SW,stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW, stopBlink4SW}},
 {{0x24,0x00},50,{stopSolidSW,stopSolidSW,stopSolidSW,stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW, stopSolidSW}},
 {{0x24,0x02},200,{goS,goW,goS,goW, goW, goW, goS, goS}}};

// Local function prototypes 
void Init_GPIO_PortsEBF(void);
 
int main(void){
  uint8_t n; // state number
  uint32_t Input;
	// Call various initialization functions
  PLL_Init(Bus80MHz);               // initialize 80 MHz system clock
  SysTick_Init();                   // initialize SysTick timer
  Init_GPIO_PortsEBF();
	// Establish initial state of traffic lights
  n = goS;                          // initial state: Green north; Red east
  GPIO_PORTF_OUT = 0x02;						//portF data is set to dont walk light is on
	while(1){
    LIGHT = FSM[n].Out[0];             // set lights to current state's Out value
		GPIO_PORTF_OUT = FSM[n].Out[1]; 
		
    SysTick_Wait10ms(FSM[n].Time);  // wait 10 ms * current state's Time value
    Input = SENSOR;                 // get new input from car detectors

    n = FSM[n].Next[Input];         // transition to next state
  }
}


void Init_GPIO_PortsEBF(void) {
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32;        // activate clock for Port E, Port F, and Port B
  // allow time for clock to stabilize
	delay = SYSCTL_RCGC2_R;

  GPIO_PORTB_DIR_R |= 0x3F;         // make PB5-0 out
  GPIO_PORTB_AFSEL_R &= ~0x3F;      // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;         // enable digital I/O on PB5-0
                                    // configure PB5-0 as GPIO
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF000000)+0x00000000;
  GPIO_PORTB_AMSEL_R &= ~0x3F;      // disable analog functionality on PB5-0
  GPIO_PORTE_DIR_R &= ~0x07;        // make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07;      // disable alt funct on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;         // enable digital I/O on PE2-0
                                    // configure PE2-0 as GPIO
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFFFF000)+0x00000000;
  GPIO_PORTE_AMSEL_R &= ~0x07;      // disable analog functionality on PE2-0
	
	//Port F initialization
	GPIO_PORTF_DIR_R |= 0x0A;         // make PF3,PF1 out
  GPIO_PORTF_AFSEL_R &= ~0x0A;      // disable alt funct on PF5-0
  GPIO_PORTF_DEN_R |= 0x0A;         // enable digital I/O on PF5-0
                                    // configure PF3, PF1 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF0F0F)+0x00000000;
  GPIO_PORTF_AMSEL_R &= ~0x0A;      // disable analog functionality on PF3, PF1
	}
