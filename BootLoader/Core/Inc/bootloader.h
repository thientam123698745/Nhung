/*
 * bootloader.h
 *
 *  Created on: Oct 16, 2020
 *      Author: vikto
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#define APP1_START (0x08040000)			//Origin + Bootloader size (20kB)
#define APP2_START (0x08060000)			//Origin + Bootloader size (20kB) + App1 Bank (22kB)


#define ERASE_FLASH_MEMORY "#$ERASE_MEM"
#define FLASHING_START "#$FLASH_START"
#define FLASHING_FINISH "#$FLASH_FINISH"
#define FLASHING_ABORT "#$FLASH_ABORT"

#include "main.h"
//#include "usbd_cdc_if.h"
#include <string.h>

typedef enum
{
    JumpMode,
	FlashMode
} BootloaderMode;

//typedef enum
//{
//    App1,
//	App2
//} AppSelection;



typedef void (application_t)(void);

typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

//int App;


//extern USBD_HandleTypeDef hUsbDeviceFS;//it is defined in the usb_device.c
void bootloaderInit();
void flashWord(uint32_t word);
uint32_t readWord(uint32_t address);
void eraseMemory();
void unlockFlashAndEraseMemory();
void lockFlash();
void jumpToApp();
void deinitEverything();
uint8_t string_compare(char array1[], char array2[], uint16_t length);
void errorBlink();
void messageHandler(uint8_t* Buf);

#endif /* INC_BOOTLOADER_H_ */
