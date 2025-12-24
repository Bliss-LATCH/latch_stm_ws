/*
 * can_device.h
 *
 *  Created on: Dec 23, 2025
 *      Author: nick
 */

#ifndef CAN_DEVICE_H_
#define CAN_DEVICE_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// FORWARD DECLARATION
typedef struct CANDevice_t CANDevice_t;

/** @breif function pointer for the rx callback
 *
 */
typedef void (*CAN_RxCallback_t)(CANDevice_t *device);

/** This will hold the current state of the device CAN
 * 	It holds the tx and rx header information, the mailbox information, and
 * 	the received buffer data
 */
typedef struct CANDevice_t {
	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	CAN_RxHeaderTypeDef rxHeader;
	uint32_t txMailbox;
	uint8_t rxBuffer[8];
	bool isInitialized; // if the Device has been initalized

	bool msgReceived; // weather or not we have received a message from can

	CAN_RxCallback_t rxCallback;
} CANDevice_t;

/** This is the message data type that holds the message information
 *  to transmit data over the bus
 */
typedef struct {

} CANMessage_t;

// device based filtering config
void can_config_filter(CANDevice_t *device, uint16_t *idList, uint16_t idCount);

// links the CAN handle to the device variable starts the notification to start the interrupt triggering of receiving data
void device_can_init(CANDevice_t *device, CAN_HandleTypeDef *hcan);

// must write the custom callback to handle the information received from CAN
void link_rx_callback(CANDevice_t *device, CAN_RxCallback_t callback);

// override for the pending fifo0 message callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// wrapper function to transmit the can data using the CAN device
void transmit_can_data(CANDevice_t *device, const uint32_t canId, const uint8_t txData[], const uint8_t txLen);

#endif /* CAN_DEVICE_H_ */
