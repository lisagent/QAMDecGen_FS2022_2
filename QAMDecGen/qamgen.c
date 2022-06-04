/*
* qamgen.c
*
* Created: 05.05.2020 16:24:59
*  Author: Chaos
*/ 
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"

#include "qaminit.h"
#include "qamgen.h"
#include "ButtonHandler.h"


EventGroupHandle_t EventGroupQAMGen;
#define LOCK_DATA	1 << 0
#define DATA_READY	1 << 1
#define LOCK_CLEARED 1 << 2
#define CREATE_SENDDATA 1 << 3


#define MODE_IDLE		0x00
#define MODE_SENDING	0x01


const int16_t sinLookup100[NR_OF_SAMPLES*2] = {0x0,0x18F,0x30F,0x471,0x5A7,0x6A6,0x763,0x7D8,
												0x7FF,0x7D8,0x763,0x6A6,0x5A7,0x471,0x30F,0x18F,
												0x0,0xFE71,0xFCF1,0xFB8F,0xFA59,0xF95A,0xF89D,0xF828,
												0xF801,0xF828,0xF89D,0xF95A,0xFA59,0xFB8F,0xFCF1,0xFE71,
												0x0,0x18F,0x30F,0x471,0x5A7,0x6A6,0x763,0x7D8,
												0x7FF,0x7D8,0x763,0x6A6,0x5A7,0x471,0x30F,0x18F,
												0x0,0xFE71,0xFCF1,0xFB8F,0xFA59,0xF95A,0xF89D,0xF828,
												0xF801,0xF828,0xF89D,0xF95A,0xFA59,0xFB8F,0xFCF1,0xFE71};
														
const int16_t sinLookup50[NR_OF_SAMPLES*2] = {0x0,0xC8,0x187,0x238,0x2D3,0x353,0x3B1,0x3EB,
												0x3FF,0x3EB,0x3B1,0x353,0x2D3,0x238,0x187,0xC8,
												0x0,0xFF38,0xFE79,0xFDC8,0xFD2D,0xFCAD,0xFC4F,0xFC15,
												0xFC01,0xFC15,0xFC4F,0xFCAD,0xFD2D,0xFDC8,0xFE79,0xFF38,
												0x0,0xC8,0x187,0x238,0x2D3,0x353,0x3B1,0x3EB,
												0x3FF,0x3EB,0x3B1,0x353,0x2D3,0x238,0x187,0xC8,
												0x0,0xFF38,0xFE79,0xFDC8,0xFD2D,0xFCAD,0xFC4F,0xFC15,
												0xFC01,0xFC15,0xFC4F,0xFCAD,0xFD2D,0xFDC8,0xFE79,0xFF38,};

#define SENDBUFFER_SIZE_IDLE		 2
#define SENDBUFFER_SIZE_SENDING		 42

SemaphoreHandle_t MutexQAMGen; //A-Ressource
uint8_t sendbuffer_IDLE[SENDBUFFER_SIZE_IDLE] = {0,3}; //P-Ressource
//uint8_t sendbuffer_SENDING[SENDBUFFER_SIZE_SENDING] = {0,1,0,1,0,1,2,1,3,0,1,1,3,2,1,0,0,1,0,1};

void vQuamGen(void *pvParameters) {	
	MutexQAMGen = xSemaphoreCreateMutex(); 
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	xEventGroupWaitBits(evDMAState, DMAGENREADY, false, true, portMAX_DELAY);
	initButtons();
	for(;;) {
			updateButtons();
			
			if(getButtonPress(BUTTON1) == SHORT_PRESSED) {
				xEventGroupSetBits(EventGroupQAMGen, CREATE_SENDDATA);
			}
			vTaskDelay(10/portTICK_RATE_MS);
}

void fillBuffer(uint16_t buffer[NR_OF_SAMPLES]) {
	static int pSendbuffer_IDLE = 0;
	static int pSendbuffer_SENDING = 0;
		int mode = MODE_IDLE;
			switch(){
				case MODE_IDLE:
					for(int i = 0; i < NR_OF_SAMPLES;i++) {
						switch(sendbuffer_IDLE[pSendbuffer_IDLE]) {
							case 0:
								buffer[i] = 0x800 + (sinLookup100[i]);
							break;
							case 1:
								buffer[i] = 0x800 + (sinLookup100[i+16]);
							break;
							case 2:
								buffer[i] = 0x800 + (sinLookup50[i]);
							break;
							case 3:
								buffer[i] = 0x800 + (sinLookup50[i+16]);
							break;
						}
					}
					if(pSendbuffer_IDLE < SENDBUFFER_SIZE_IDLE-1) {
						pSendbuffer_IDLE++;
					} else {
						pSendbuffer_IDLE = 0;
				}	
				break;
				
				case MODE_SENDING:	
					for(int i = 0; i < NR_OF_SAMPLES;i++) {
						switch(sendbuffer_SENDING[pSendbuffer_SENDING]) {
							case 0:
								buffer[i] = 0x800 + (sinLookup100[i]);
							break;
							case 1:
								buffer[i] = 0x800 + (sinLookup100[i+16]);
							break;
							case 2:
								buffer[i] = 0x800 + (sinLookup50[i]);
							break;
							case 3:
								buffer[i] = 0x800 + (sinLookup50[i+16]);
							break;
						}
					}
					if(pSendbuffer_SENDING < SENDBUFFER_SIZE_SENDING-1) {
							pSendbuffer_SENDING++;
						} else {
							pSendbuffer_SENDING = 0;
					}
					break;
		}
}
uint8_t sendbuffer_SENDING[100]
uint8_t SendID[100]
uint8_t checksum[20]
void Create_Send_Data(){	
		if((xEventGroupGetBits(EventGroupQAMGen)&StartTask)){		
			case MODE_SENDING;
				if(xSemaphoreTake(MutexQAMGen, 10/portTICK_RATE_MS) == pdTRUE) {
					SendID++;
					char senddata[10] = "HelloWorld";
					uint8_t datalen = strlen(senddata);
					//Start-Signal
					sendbuffer_SENDING[0] =1;
					sendbuffer_SENDING[1] =2;
					//ID
					sendbuffer_SENDING[2] =SendID & 0x03;
					sendbuffer_SENDING[3] =(SendID >> 2 ) & 0x03;
					//Lenght
					sendbuffer_SENDING[4] =(senddata >> 0 ) & 0x03;
					sendbuffer_SENDING[5] =(senddata >> 2 ) & 0x03;
					sendbuffer_SENDING[6] =(senddata >> 4 ) & 0x03;
					sendbuffer_SENDING[7] =(senddata >> 6 ) & 0x03;
					//Daten
					for (int i = 0; i<datalen; i++){
						sendbuffer_SENDING[8+i*4+0] = (senddata[i] >>0) & 0x03;
						sendbuffer_SENDING[8+i*4+1] = (senddata[i] >>2) & 0x03;
						sendbuffer_SENDING[8+i*4+2] = (senddata[i] >>3) & 0x03;
						sendbuffer_SENDING[8+i*4+3] = (senddata[i] >>4) & 0x03;
					}
					//Checksumme
					for(int i = 0; i<7+(datalen*4); i++) {
						checksum += sendbuffer_SENDING[i];
					}
					sendbuffer_SENDING[(i<7+(datalen*4))+0] = checksum >> 0 &0x03;
					sendbuffer_SENDING[(i<7+(datalen*4))+1] = checksum >> 2 &0x03;
					sendbuffer_SENDING[(i<7+(datalen*4))+2] = checksum >> 4 &0x03;
					sendbuffer_SENDING[(i<7+(datalen*4))+3] = checksum >> 6 &0x03;
					xSemaphoreGive(MutexQAMGen);
				}
			break;
		}

ISR(DMA_CH0_vect)
{
	//static signed BaseType_t test;
	
	DMA.CH0.CTRLB|=0x10;
	fillBuffer(&dacBuffer0[0]);
}

ISR(DMA_CH1_vect)
{
	DMA.CH1.CTRLB|=0x10;
	fillBuffer(&dacBuffer1[0]);
}
