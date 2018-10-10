
 /*
 * speeddetector.h
 *
 *  Created on: Dec 18, 2017
 *      Author: Srivishnu Alvakonda
                Divya Sampath Kumar
 */

//#include "em_device.h"
#include "em_adc.h"
//#include "em_chip.h"
//#include "em_cmu.h"
//#include "em_emu.h"
#include "rtcdriver.h"
#include "graphics.h"
#include "bspconfig.h"
#include <stddef.h>
#include "speeddetector.h"

/***************************************************************************//**
 * Local defines
 ******************************************************************************/

/** Time (in ms) between periodic updates of the measurements. */
#define PERIODIC_UPDATE_MS      100

/***************************************************************************//**
 * Local variables
 ******************************************************************************/


int sensorcount=1;
float time_count=0;
int signal;



static void timestamp_init(void)
{
  /* Enable debug clock AUXHFRCO */
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= CoreDebug_DHCSR_C_DEBUGEN_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009 | ITM_TCR_DWTENA_Pos;

  /* Enable PC and IRQ sampling output */
  DWT->CTRL = 0x400113FF;
  DWT->CYCCNT = 0;
}

static inline uint32_t clk_timestamp_get(void)
{
  return DWT->CYCCNT;
}
/***************************************************************************//**
 * @brief Initialize ADC for temperature sensor readings in single poin
 ******************************************************************************/
static void AdcSetup(void)
{
  /* Enable ADC clock */
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef       init  = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  //init.prescale = ADC_PrescaleCalc(400000, 0);
  init.prescale = ADC_PrescaleCalc(42000, 0);
  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  sInit.reference   = adcRefVDD;
  sInit.acqTime     = adcAcqTime128; /* Minimum time for temperature sensor */
  //sInit.posSel      = adcPosSelTEMP;
  if(sensorcount == 1)
  {
  sInit.posSel      = adcPosSelAPORT3XCH10 ;
  }
  else
  {
	  sInit.posSel      = adcPosSelAPORT3XCH8 ;
  }
  ADC_InitSingle(ADC0, &sInit);
}

/***************************************************************************//**
 * @brief  Do one ADC conversion
 * @return ADC conversion result
 ******************************************************************************/
static uint32_t US1_Read(void)
{
  ADC_Start(ADC0, adcStartSingle);
  //ADC_Start(ADC0,adcStartScanAndSingle) ;
  while ( (ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0 ) {
  }
  return ADC_DataSingleGet(ADC0);
}


static uint32_t US2_Read(void)
{
  ADC_Start(ADC0, adcStartSingle);
  //ADC_Start(ADC0,adcStartScanAndSingle) ;
  while ( (ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0 ) {
  }
  return ADC_DataSingleGet(ADC0);
}



void CMU_Setup(void){

    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);    /* Setting HFRCO as HFCLK system clock */
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);    /* Insure HFXO is disabled to save energy */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable Low Frequency Clock for Low Frequency Peripherals */
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
    CMU_ClockEnable(cmuClock_CORELE,true);

    /* Enabling clocks to the desired peripherals */
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_I2C0, true);
    CMU_ClockEnable(cmuClock_I2C1, true);
    CMU_ClockEnable(cmuClock_USART0, true);
    //CMU_ClockEnable(cmuClock_LEUART0, true);
}

void i2c0Setup(void)
{

    I2C_Init_TypeDef I2C0_init = I2C_INIT_DEFAULT;
    int intFlags;

    I2C0_init.enable = true;
    I2C0_init.master = true;
    I2C0_init.refFreq = 0;
    I2C0_init.freq = I2C_FREQ_FAST_MAX;                    /* Max I2C bus of NFC tag */
    I2C0_init.clhr = i2cClockHLRStandard;

    I2C0->CTRL &= ~I2C_CTRL_GCAMEN;
    I2C0->CTRL &= ~I2C_CTRL_ARBDIS;


    I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK) &(~_I2C_ROUTELOC0_SDALOC_MASK)) | (I2C_ROUTELOC0_SCLLOC_LOC19) | (I2C_ROUTELOC0_SDALOC_LOC13);


    /* Initialize and enable I2C0 port */
    I2C_Init(I2C0, &I2C0_init);


    /* Exit the busy state.  The I2C will be in this state out of RESET */
    if (I2C0->STATE & I2C_STATE_BUSY){
        I2C0->CMD = I2C_CMD_ABORT;
    }

    intFlags = I2C0->IF;
    I2C0->IFC = intFlags;

    I2C0->IEN = 0;

     /* Clear and enable interrupt from I2C module */
     NVIC_ClearPendingIRQ(I2C0_IRQn);
     NVIC_EnableIRQ(I2C0_IRQn);
}


void NFC_Read(unsigned int mema, unsigned int *buffer)
{
		int i;
        I2C0->CTRL |= I2C_CTRL_AUTOACK;
        I2C0->TXDATA = SLAVE_ADDR;
        I2C0->CMD = I2C_CMD_START;

        while ((I2C0->IF & I2C_IF_ACK) == 0);
        I2C0->IFC = I2C_IF_ACK;

        I2C0->TXDATA = mema;

        while ((I2C0->IF & I2C_IF_ACK) == 0);
        I2C0->IFC = I2C_IF_ACK;

        I2C0->CMD        = I2C_CMD_START;
        I2C0->TXDATA     = (SLAVE_ADDR<<1)|NFC_READ;    // Set read bit

        while ((I2C0->IF & I2C_IF_ACK) == 0);
        I2C0->IFC = I2C_IF_ACK;

        for (i = 0; i < 16; i++)
        {
            while ((I2C0->IF & I2C_IF_RXDATAV) == 0);
            *buffer = I2C0->RXDATA;
            buffer++;
            if (i == (16 - 2)){
                I2C0->CTRL &= ~I2C_CTRL_AUTOACK;
            }
        }
        I2C0->CMD = I2C_CMD_ACK;
        I2C0->CMD = I2C_CMD_STOP;
        I2C0->IFC = I2C_IFC_ACK;

    }



void NFC_Write(unsigned int mema, unsigned int *buffer)
{
		int i;

        I2C0->TXDATA = SLAVE_ADDR;
        I2C0->CMD = I2C_CMD_START;

        while ((I2C0->IF & I2C_IF_ACK) == 0);
        I2C0->IFC = I2C_IF_ACK;

        I2C0->TXDATA = mema;

        while ((I2C0->IF & I2C_IF_ACK) == 0);
        I2C0->IFC = I2C_IF_ACK;

        for (i = 0; i < 16; i++)
        {
            I2C0->TXDATA = *buffer;
            while ((I2C0->IF & I2C_IF_ACK) == 0);
            I2C0->IFC = I2C_IF_ACK;
            buffer++;
            if (i == (16 - 2))
            {
            	I2C0->CTRL &= ~I2C_CTRL_AUTOACK;
            }

        }
        I2C0->CMD=I2C_CMD_NACK;
        I2C0->CMD=I2C_CMD_STOP;
        for(long int i=0;i<500;i++);
}

void NFC_On(void)
{
	int i;
    GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeWiredAndPullUp, 1);

    //Time for SCL and  SDA to pull up to VOUT=3.3V
    for (int i = 0; i < 50000; i++);

    /* Toggle I2C SCL 9 times to reset any I2C slave that may require it */
    for (int i=0;i<9;i++) {
    	GPIO_PinOutClear(I2C0_SCL_Port, I2C0_SCL_Pin);
    	GPIO_PinOutSet(I2C0_SCL_Port, I2C0_SCL_Pin);
    }

    i2c0Setup();
    //Time for SCL and  SDA to pull up to VOUT=3.3V
    for (i = 0; i < 50000; i++);

    /*Toggle I2C SCL 9 times to reset any I2C slave that may require it */
    for (int i=0;i<9;i++) {
        GPIO_PinOutClear(I2C0_SCL_Port, I2C0_SCL_Pin);
        GPIO_PinOutSet(I2C0_SCL_Port, I2C0_SCL_Pin);
    }
}

void NFC_Off(void) {

    /* Tri-state or disable I2C pins first */

    GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeDisabled, 0);            // Do not set up pull-up
    GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeDisabled, 0);            // Do not set up pull-up


}

void Nfc_I2c_Test(void)
{
    int i;

    for (i = 0; i < 16; i++) {
    	xbee_to_nfc_write_buffer[i] = 0x42;
    }
    NFC_Write(0x0A+1, xbee_to_nfc_write_buffer);
}


/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_STK_DEFAULT;
  float US1Read=15.0;
  float US2Read=15.0;
  int lock_count=0;
  int got=143;
  float miles = 0.0008;
  float speed;
  float error_correction_below = 0.5;
  float error_correction_above = 0.5;
  float speed_limit = 3.0;
  unsigned int speed_arr[16];
  int i=0;
  int x;
  //float celsius;

  /* Chip errata */
  CHIP_Init();
  CMU_Setup();


  /* Init DCDC regulator and HFXO with WSTK radio board specific parameters
     kits\SLSTK3402A_EFM32PG\config\bspconfig.h. */
 /* EMU_DCDCInit(&dcdcInit);
  CMU_HFXOInit(&hfxoInit);

  /* Switch HFCLK to HFXO and disable HFRCO */
  /*
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
*/
  /* Setup ADC */

  while (true)
  {
		    while(i<15)
		    {
		    	speed_arr[i]= 0x46;
		    	i++;
		    }
	        sensorcount = 1;
	        GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 0);
	        GPIO_PinModeSet(gpioPortB, 13, gpioModePushPull, 1);
	        wait(1000);
		    AdcSetup();
	        US1Read = US1_Read();
	        US1Read = US1Read / 195.0 ;

	  if(US1Read < 2.2)
	  {
		  GPIO_PinModeSet(gpioPortB, 13, gpioModePushPull, 0);
		  timestamp_init();
		  US1Read=15.0;
		  US2Read=15.0;
		  lock_count=0;
		  sensorcount = 2;
		  GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 1);
		  wait(1000);
		  AdcSetup();
		  while(lock_count < 1000)
		  {
		      US2Read = US2_Read();
	      	  US2Read = US2Read /195.0;
		  if(US2Read < 1.4)
		    {
			  GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 0);
			  time_count = clk_timestamp_get();
			  time_count = time_count/100000;
              time_count = time_count/281;
              time_count = time_count/3600;
              speed = miles/time_count;
              if(speed > speed_limit)
              {
            	  speed = speed-error_correction_above;
            	  if(speed > speed_limit)
            	  {
            		  //signal = 11111;

            		  speed_arr[i] = (int)speed;
            		  //speed_arr[i] = 0x42;
            		  i = i+1;

            		  if(i==16)
            		  	   {
            		  	     NFC_On();
            		  	     NFC_Write(0x0A+1, speed_arr);
            		  	     i=0;
            		  	   }
            		  	wait(1000);
            		  	//NFC_Off();
            		  signal = 11;
            	  }
            	  else
            	  {
            		  //signal = 00000;
            		  speed_arr[i] = (int)speed;
            		  //speed_arr[i] = 0x42;
            		  i = i+1;

            		  if(i==16)
            		  {
            		     NFC_On();
            		     NFC_Write(0x0A+1, speed_arr);
            		     i=0;
            		  }
            		  wait(1000);
            		  //NFC_Off();
            		  signal = 22;
            	  }
            	  // Nfc_I2c_Test();
            	 // NFC_Off();

              }
              else
              {
            	  speed = speed-error_correction_below;
            	  if(speed > speed_limit)
            	  {
            		  speed_arr[i] = (int)speed;
            		  i = i+1;
            		  signal = 11;

            		  if(i==16)
            		  	   {
            		  	     NFC_On();
            		  	     NFC_Write(0x0A+1, speed_arr);
            		  	     i=0;
            		  	   }
            		  	   wait(1000);
            		  	   //NFC_Off();
            	  }
            	  else
            	  {
            		  speed_arr[i] = (int)speed;
            		  i = i+1;

            		  if(i==16)
            		  {
            		     NFC_On();
            		     NFC_Write(0x0A+1, speed_arr);
            		     i=0;
            		     }
            		     wait(1000);
            		     //NFC_Off();
            		  signal = 22;
            	  }
            	  //NFC_On();
            	  //NFC_Write(0x0A+1, &signal);
            	 // NFC_Off();

              }

			  x=0;
			  while(x<1)
			  {
			  x=x+1;

			  }
			  sensorcount = 1;
			  break;
		    }
		  lock_count++;
		  }
	   }
	  /*if(i==16)
	   {
	     NFC_On();
	     NFC_Write(0x0A+1, speed_arr);
	     i=0;
	   }*/
    EMU_EnterEM2(false);
  }//while_main_end

}//main_end


