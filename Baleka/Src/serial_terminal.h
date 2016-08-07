/*
 * SerialTerminal.h
 *
 *  Created on: Jul 4, 2012
 *      Author: James Gowans
 *
 *      Description:  Provides functionality for printf and console via UART
 */


#ifndef SERIALTERMINAL_H_
#define SERIALTERMINAL_H_

#include "stm32f10x.h"
#include "string.h"
#define TX_BUFFER_SIZE 50


typedef struct {
  uint8_t data[TX_BUFFER_SIZE - 2];

  uint16_t bytes_to_tx;
} CommsTask_TransmitPacketStruct;

uint8_t dataS1[TX_BUFFER_SIZE];
uint8_t dataS2[TX_BUFFER_SIZE];
uint8_t dataS3[TX_BUFFER_SIZE];
uint8_t dataS4[TX_BUFFER_SIZE];
uint16_t Tx_chars;
uint8_t TxBuff[TX_BUFFER_SIZE];
uint16_t lengthS1;

void serialTerminal_Init();

void serialTerminal_packetize(uint8_t* payload_to_packS1,uint8_t* payload_to_packS2,uint8_t* payload_to_packS3,uint8_t* payload_to_packS4, uint16_t length_of_payloadS1,uint16_t length_of_payloadS2,uint16_t length_of_payloadS3,uint16_t length_of_payloadS4);

void SerialTerminal_BeginTx(void);

#endif /* SERIALTERMINAL_H_ */
