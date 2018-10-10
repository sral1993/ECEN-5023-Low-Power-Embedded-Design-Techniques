/*
 * speeddetector.h
 *
 *  Created on: Dec 18, 2017
 *      Author: Srivishnu Alvakonda
                Divya Sampath Kumar
 */

#ifndef SRC_SPEEDDETECTOR_H_
#define SRC_SPEEDDETECTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_dma.h"
#include "dmactrl.h"

/* sleep mode enumerations */
#define    EM0    0
#define    EM1    1
#define    EM2    2
#define    EM3    3
#define    EM4    4

/*sleep mode count state*/
unsigned int LowestEnergyMode[EM4+1];


#define I2C0_SCL_Port        gpioPortA

#define I2C0_SCL_Pin         2
#define I2C0_SDA_Port        gpioPortC
#define I2C0_SDA_Pin         8

/* NFC I2C register definitions */
#define SLAVE_ADDR        	  0xAA
#define NFC_READ        	  0x01
#define NFC_WRITE			  0x00




void Sleep(void);
void BlockSleep(unsigned int energymode);
void UnBlockSleep(unsigned int energymode);
void CMU_Setup(void);


void i2c0Setup(void);
void NFC_On(void);
void NFC_Off(void);
void NFC_Read(unsigned int mema, unsigned int *buffer);
void NFC_Write(unsigned int mema, unsigned int *buffer);

void Nfc_I2c_Test(void);

#define NFC_BUFFER_SIZE   16
unsigned nfc_read_buffer[NFC_BUFFER_SIZE];
unsigned int xbee_to_nfc_write_buffer[NFC_BUFFER_SIZE];

#endif /* SRC_SPEEDDETECTOR_H_ */
