/*
 * can_device.h
 *
 *  Created on: Dec 23, 2025
 *      Author: nick
 */

#ifndef CAN_DEVICE_H_
#define CAN_DEVICE_H_

#include "stm32f4xx_hal_conf.h"
#include <stdbool.h>

/** This will hold the current state of the device CAN
 * 	It holds the tx and rx header information, the mailbox information, and
 * 	the received buffer data
 */
typedef struct {
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	CAN_RxHeaderTypeDef rxHeader;
	uint32_t txMailbox;
	uint8_t rxBuffer[8];
	bool isInitialized;
} CANDevice_t;

/** This is the message data type that holds the message information
 *  to transmit data over the bus
 */
typedef struct {

} CANMessage_t;


void configure_filter(CAN_HandleTypeDef *hcan, uint16_t *idList, uint16_t idCount);


#endif /* CAN_DEVICE_H_ */
