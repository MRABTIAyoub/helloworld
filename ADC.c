#include "stm32f4xx.h"                  // Device header
#include "math.h"

void Configuration();
int distance=0;
void ADC_Init (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Common Control Register (CCR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/
	
//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	RCC->AHB1ENR |= (1<<0);  // enable GPIOA clock
	
//2. Set the prescalar in the Common Control Register (CCR)	
	ADC->CCR |= 1<<16;  		 // PCLK2 divide by 4
	
//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	
//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 |= (1<<1);     // enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOC after each conversion
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
	
//5. Set the Sampling Time for the channels	
	ADC1->SMPR2 &= ~((1<<3) | (1<<12));  // Sampling time of 3 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (1<<20);   // SQR1_L =1 for 2 conversions
	
//7. Set the Respective GPIO PINs in the Analog Mode	
	GPIOA->MODER |= (3<<2);  // analog mode for PA 1 (chennel 1)
	GPIOA->MODER |= (3<<8);  // analog mode for PA 4 (channel 4)
}


void ADC_Enable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us) 
	************************************************/
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000;
	while (delay--);
}


void ADC_Start (int channel)
{
	/************** STEPS TO FOLLOW *****************
	1. Set the channel Sequence in the SQR Register
	2. Clear the Status register
	3. Start the Conversion by Setting the SWSTART bit in CR2
	************************************************/
	
	
/**	Since we will be polling for each channel, here we will keep one channel in the sequence at a time
		ADC1->SQR3 |= (channel<<0); will just keep the respective channel in the sequence for the conversion **/
	
	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);    // conversion in regular sequence
	
	ADC1->SR = 0;        // clear the status register
	
	ADC1->CR2 |= (1<<30);  // start the conversion
}


void ADC_WaitForConv (void)
{
	/*************************************************
	EOC Flag will be set, once the conversion is finished
	*************************************************/
	while (!(ADC1->SR & (1<<1)));  // wait for EOC flag to set
}

uint16_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}

void ADC_Disable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Disable the ADC by Clearing ADON bit in CR2
	************************************************/	
	ADC1->CR2 &= ~(1<<0);  // Disable ADC
}


uint16_t ADC_VAL = 0;



int main ()
{
	float distance;
	Configuration();
	ADC_Init ();
	ADC_Enable ();	
	
	while (1)
	{
		ADC_Start (1);
		ADC_WaitForConv ();
		ADC_VAL = ADC_GetVal();
		
		distance  = ADC_VAL* 5.0 / 1023.0;  // value from sensor * (3.6/1024)
   // distance = 13*pow(volts, -1); // worked out from datasheet graph
//	float volts = ADC_VAL*0.00087890625;  // value from sensor * (3.6/1024)


  
  if (distance <= 1 ){
			GPIOG->ODR |= (1<<13) ;
			GPIOG->ODR &=~ (1<<14);

		//  GPIOD->ODR |= (1<<15) ;
  }
  if (distance >= 1 ){
			GPIOG->ODR |= (1<<14) ;
			GPIOG->ODR &=~ (1<<13);

		//  GPIOD->ODR |= (1<<15) ;
  }
	
	
	/*	ADC_Start (4);
		ADC_WaitForConv ();
		ADC_VAL[1] = ADC_GetVal();*/
	}
	
}
void Configuration(){
	RCC->AHB1ENR |= ((1<<6) |(1<<0));
	GPIOG->MODER &=~((1<<29)|(1<<27));
	GPIOG->MODER |=((1<<28)|(1<<26));
	GPIOG->OTYPER &=~((1<<13) | (1<<14));
	GPIOG->OSPEEDR &=~((1<<29)|(1<<27));
	GPIOG->OSPEEDR |=((1<<28)|(1<<26));

}






/*#include "stm32f4xx.h"                  // Device header
#include "math.h"
int data;
void Configuration();
int main (void){



Configuration();
while(1){
ADC1->CR2 |=(1<<0);
while(!(ADC1->SR &(1<<1))){}	
	data=ADC1->DR;
	
	float volts = data*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  	
  if (distance <= 8 ){
			GPIOD->ODR |= (1<<13) ;
		  GPIOD->ODR |= (1<<15) ;
  }
	else {
			GPIOD->ODR &=~ (1<<13);
			GPIOD->ODR &=~ (1<<15);
	}
	}	
}

void Configuration(){
	RCC->APB2ENR |=(1<<8);
	RCC->AHB1ENR |= ((1<<3) |(1<<0));
	GPIOD->MODER &=~((1<<31)|(1<<27));
	GPIOD->MODER |=((1<<30)|(1<<26));
	GPIOD->OTYPER &=~((1<<13) | (1<<15));
	GPIOD->OSPEEDR &=~((1<<31)|(1<<27));
	GPIOD->OSPEEDR |=((1<<30)|(1<<26));
	ADC->CCR |=(1<<17);
	ADC->CCR &=~(1<<16);
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	ADC1->CR2 |=(1<<0);
	ADC1->CR2 = (1<<1);     // enable continuous conversion mode
  ADC1->CR2 |= (1<<10);    // EOC after each conversion
  ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
// Enable DMA for ADC
  ADC1->CR2 |= (1<<8);
// Enable Continuous Request
  ADC1->CR2 |= (1<<9);
	ADC1->SMPR2 &= ~((7<<3) | (7<<12));
	ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions
	ADC1->SQR3 |=(1<<0);	
	GPIOA->MODER |=((1<<3) | (1<<2));	


}*/