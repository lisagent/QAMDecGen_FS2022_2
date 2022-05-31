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

//void Interface(void *pvParameters);

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
uint8_t sendbuffer_SENDING[SENDBUFFER_SIZE_SENDING] = {0,1,0,1,0,1,2,1,3,0,1,1,3,2,1,0,0,1,0,1};


//Es soll eine StateMachine erstellt werden, mit deren Hilfe die einzelnen States ablaufen. 
//Sie soll aus folgenden State bestehen: Idle, Start, ID,Lenght,Data,Checksumme
//Stand 24.05.22 Die StateMAchine soll nicht so genau definiert sein. Es soll eine StateMachine im fillBuffer gemacht werden
//Diese soll nur die zwei Zustände Idle und Sending haben. Im Sending werden alle Daten von Start,ID,Lengt,... zusammen geschrieben
//Es muss noch einen Extratask haben, zb direkt im vQuamGen. Der besteht aus Taste und Encoder. Es kann die gleiche Tastenabfrage wie bei den anderen Projekten genommen werden
//Wenn die Taste gedrückt wird, werden alle Daten durch den Encoder in den SendingBuffer geschrieben. Durch ein Semaphore soll dann noch sicher gestellt werden, dass die Daten
//Nur im Idle-Zustand gesendet werden. Wenn er grade im Sending-State ist dürfen keine Daten übertragen werden.

//Im fillBuffer muss der ganze Kram nochmal copy_paste gemacht werden für die zwei state. Es wird eine State-Maschine erstellt, mit der dann im fillBuffer zwischen
//IDLE und SENDING gewechselt wird. Im IDLE kann unten das ++ so weg gemacht werden, dass er immer nur zwischen 0,3 toggelt.
//Im SENDING soll es so gemacht werden wie im SENDBUFFER, mit der [] wird gesagt an welcher Stelle er welchen index haben soll. ALso zb als index [5] wird
// das 5te SYmbol gesendet. Dan stelle [6] das sechste. Dabei ist es sehr wichtig zu defieren, wie lang das gesssamte senden ist
// Mit einem MUTEX kann dann noch gesagt werden, dass er erst wieder Daten übernehmen darf, wenn er mit dem Senden fertig ist.

void vQuamGen(void *pvParameters) {
	
	MutexQAMGen = xSemaphoreCreateMutex(); //Muss wahrscheinlich alle mit ins Main
	//EventGroupQAMGen = xEventGroupCreate();
	//xEventGroupSetBits(EventGroupQAMGen, ALGO1);
	
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
			
			//sendbuffer[SENDBUFFER_SIZE_IDLE] = {0,3};
			if((xEventGroupGetBits(EventGroupQAMGen) & LOCK_DATA) == LOCK_DATA) {
				xEventGroupClearBits(EventGroupQAMGen, LOCK_DATA);
				sendbuffer[SENDBUFFER_SIZE_IDLE] = 0;
			}
			if(xSemaphoreTake(MutexQAMGen, 10/portTICK_RATE_MS) == pdTRUE) {
				sendbuffer[SENDBUFFER_SIZE_IDLE] = localx;
				xSemaphoreGive(MutexQAMGen);
			}
			vTaskDelay(10/portTICK_RATE_MS);
	}
	//xTaskCreate(Interface, NULL, configMINIMAL_STACK_SIZE+500, NULL, 2, NULL);

		}
	}
}

void fillBuffer(uint16_t buffer[NR_OF_SAMPLES]) {
	static int pSendbuffer_IDLE = 0;
	static int pSendbuffer_SENDING = 0;
	
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
	if(pSendbuffer_SENDING < SENDBUFFER_SIZE_IDLE-1) {
		pSendbuffer_IDLE++;
	} else {
		pSendbuffer_IDLE = 0;
	}	
	
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
}
//switch(mode)
//case MODE_IDLE:
//if(xSemaphoreTake(MutexQAMGen, 10/portTICK_RATE_MS) == pdFALSE) {
	//uint8_t sendbuffer[SENDBUFFER_SIZE_IDLE] = {0,3};
	//break;
uint8_t Symbuffer[100]
uint8_t SendID[100]
void Create_Send_Data(){	
		if((xEventGroupGetBits(EventGroupQAMGen)&StartTask)){		
			case MODE_SENDING
			if(xSemaphoreTake(MutexQAMGen, 10/portTICK_RATE_MS) == pdTRUE) {
				SendID++;
				char senddata[10] = "HelloWorld";
				uint8_t datalen = strlen(senddata)
				sendbuffer[0] =1
				sendbuffer[1] =2
				sendbuffer[2] =SendID & 0x03
				sendbuffer[3] =(SendID >> 2 ) & 0x03
				sendbuffer[4] =(senddata >> 0 ) & 0x03
				sendbuffer[5] =(senddata >> 2 ) & 0x03
				sendbuffer[6] =(senddata >> 4 ) & 0x03
				sendbuffer[7] =(senddata >> 6 ) & 0x03
				for (int i = 0; i<datalen; i++){
					sendbuffer[8+i*4+0] = (senddata[i] >>0) & 0x03;
					sendbuffer[8+i*4+1] = (senddata[i] >>2) & 0x03;
					sendbuffer[8+i*4+2] = (senddata[i] >>3) & 0x03;
					sendbuffer[8+i*4+3] = (senddata[i] >>4) & 0x03;
				}
				for(int i = 0; i<7+(datalen*4); i++) {
					checksum += sendbuffer[i];										
				}
				sendbuffer[(i<7+(datalen*4))+0] = checksum >> 0 &0x03;
				sendbuffer[(i<7+(datalen*4))+1] = checksum >> 2 &0x03;
				sendbuffer[(i<7+(datalen*4))+2] = checksum >> 4 &0x03;
				sendbuffer[(i<7+(datalen*4))+3] = checksum >> 6 &0x03;
				
				
				
				sendbuffer[3] =1
				uint8_t Symbuffer[100]
					symbuffer [0} = 
				
				uint8_t Send_Start[2] = {1,2};
				uint8_t Send_ID[2] = {0,0};
				uint8_t SendLenght[4] = {0,0,2,3};
				uint8_t SendData[32] = {1,1,0,1, 1,3,1,1, 1,2,0,1, 1,2,3,1, 0,2,0,0, 1,1,0,0, 1,2,3,0, 1,3,1,1, 1,2,3,2, 1,2,1,0, 1,2,1,1, 1,3,0,2};
				uint8_t SendChecksumme[42] = Alle Daten zusammen
				uint8_t sendbuffer[SENDBUFFER_SIZE_IDLE] = {1,2,ID,0,0,2,3, 1,1,0,1, 1,3,1,1, 1,2,0,1, 1,2,3,1, 0,2,0,0, 1,1,0,0, 1,2,3,0, 1,3,1,1, 1,2,3,2, 1,2,1,0, 1,2,1,1, 1,3,0,2, Checksumme};
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


//void StateMachine(void *pvParameters){   //muss noch implementiert werden
//	int mode = MODE_IDLE;
//		for(;;){
//			switch(mode)
//				case MODE_IDLE:
//					uint8_t sendbuffer[SENDBUFFER_SIZE_1] = {0,3};
//					if (Button1 = 1 ) {
//					mode = MODE_IDLE;
//if(pSendbuffer < SENDBUFFER_SIZE_1-1) {
//pSendbuffer++;
//} else {
//pSendbuffer = 0;
//}
//}
//if (Button1 = 0) {
//mode = MODE_START;
//if(pSendbuffer < SENDBUFFER_SIZE_1-1) {
//pSendbuffer++;
//} else {
//pSendbuffer = 0;
//}
//}
//break;
//case MODE_START:
//uint8_t sendbuffer[SENDBUFFER_SIZE_1] = {1,2};
//if(pSendbuffer < SENDBUFFER_SIZE_1-1) {
//pSendbuffer++;
//mode = MODE_START;
//} else {
//pSendbuffer = 0;
//mode = MODE_ID;
//}
//break;
//case MODE_ID:
//uint8_t sendbuffer[SENDBUFFER_SIZE_2] = {Daten, die im ID gesendet werden sollen. Sollen spaeter mal veraenderbar sein, um die Checksumme zu kontrollieren};
//if(pSendbuffer < SENDBUFFER_SIZE_2-1) {
//pSendbuffer++;
//mode = MODE_ID;
//} else {
//pSendbuffer = 0;
//mode = MODE_LENGHT;
//}
//break;
//case MODE_LENGHT
//uint8_t sendbuffer[SENDBUFFER_SIZE_2] = {0,0,2,3};
//if(pSendbuffer < SENDBUFFER_SIZE_2-1) {
//pSendbuffer++;
//mode = MODE_LENGHT;
//} else {
//pSendbuffer = 0;
//mode = MODE_DATA;
//}
//case MODE_DATA:
//uint8_t sendbuffer[SENDBUFFER_SIZE_3] = {1,1,0,1, 1,3,1,1, 1,2,0,1, 1,2,3,1, 0,2,0,0, 1,1,0,0, 1,2,3,0, 1,3,1,1, 1,2,3,2, 1,2,1,0, 1,2,1,1, 1,3,0,2};
//if(pSendbuffer < SENDBUFFER_SIZE_2-1) {
//pSendbuffer++;
//mode = MODE_DATA;
//} else {
//pSendbuffer = 0;
//mode = MODE_CHECKSUMME;
//}
//break;
//case MODE_CHECKSUMME
//uint8_t sendbuffer[SENDBUFFER_SIZE_2] = {alle Daten zusammen gezaehlt};
//if(pSendbuffer < SENDBUFFER_SIZE_2-1) {
//pSendbuffer++;
//mode = MODE_CHECKSUMME;
//} else {
//pSendbuffer = 0;
//mode = MODE_IDLE;
//}
//}
//}