/*
 * can_device.c
 *
 *  Created on: Dec 23, 2025
 *      Author: nick
 */

#include "can_device.h"

static CANDevice_t *registered_can_device = NULL;

/** @breif Configure what ids we can to look for on the CAN bus, must be called before device init
 *  @param hcan can handle to configure (should be can1)
 * 	@param idList list of ids that
 * 	@param idCount must be between 0 and (28*4 - 1)
 */
void can_config_filter(CANDevice_t *device, uint16_t *idList, uint16_t idCount) {
	CAN_FilterTypeDef filter;

	filter.SlaveStartFilterBank = 28; // allows for CAN1 to use all available filter banks
	filter.FilterActivation = CAN_FILTER_ENABLE; // enables the filter for the device
	filter.FilterScale = CAN_FILTERSCALE_16BIT; // allows for 4 filter ids per bank
	filter.FilterMode = CAN_FILTERMODE_IDLIST; // id list moode allows for multiple specific ids to be filtered
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // use the first fifo buffer, can hold 3 messages

	/* loop through all the ids in the list, and select the right filter bank using
	 * intentional integer flooring
	 */
	for(int i = 0; i < idCount; i++) {
		filter.FilterBank = i / 4;

		uint16_t shifted_id = idList[i]<<5; // left shifted by 5 since thats where the id bits start

		// determines what filter id section to place it in since we have 4 per bank
		switch (i % 4)
		{
		case 0: filter.FilterMaskIdHigh = shifted_id;
			break;
		case 1: filter.FilterMaskIdLow = shifted_id;
			break;
		case 2: filter.FilterIdHigh = shifted_id;
			break;
		case 3: filter.FilterIdLow = shifted_id;
			break;
		}

		if ((i % 4 == 3) || (i == idCount - 1)) {
			if (HAL_CAN_ConfigFilter(device->hcan, &filter) != HAL_OK){
				Error_Handler();
			}

			filter.FilterIdHigh = 0;
			filter.FilterIdLow = 0;
			filter.FilterMaskIdHigh = 0;
			filter.FilterMaskIdLow = 0;
		}
	}

};

/** @breif Start the interrupt for the receive callback and link the device hcan to the generated one
 *  @param device the CAN device variable made
 *  @param hcan can handle to configure (should be can1)
 */
void device_can_init(CANDevice_t *device, CAN_HandleTypeDef *hcan) {
	// link the device and link the CAN handle to the generated stm32cubeMX handle
	registered_can_device = device;
	device->hcan = hcan;

	// set initialized device to true
	device->isInitialized = true;

	// clear the rx buffer
	for(int i = 0; i < 8; i++) {
		device->rxBuffer[i] = 0;
	}

	// start the CAN communication
	if (HAL_CAN_Start(hcan) != HAL_OK) {
		Error_Handler();
	}

	// start the interrupt handler for receive pending requests
	if(HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}
};

/** @breif links the CAN device rxCallback to the written custom one by user
 *  @param device the CAN device variable made
 *  @param callback the callback to be linked to the device struct
 */
void link_rx_callback(CANDevice_t *device, CAN_RxCallback_t callback) {
	device->rxCallback = callback;
};

// overriding the callback here to make this easier to implement since this will be on every board
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// check if we have already created the device instance and it its using the right can
	if (registered_can_device != NULL && registered_can_device->hcan == hcan) {
		if (HAL_CAN_GetRxMessage(registered_can_device->hcan, CAN_RX_FIFO0,
								 &(registered_can_device->rxHeader),
								 registered_can_device->rxBuffer) == HAL_OK) {

			registered_can_device->msgReceived = true;

			if (registered_can_device->rxCallback != NULL) {
				registered_can_device->rxCallback(registered_can_device);
			}

		}
	}
};

/** @breif Function to transmit CAN data to the CAN bus, utilizes the HAL lib
 *  @param device the can device that holds the state information
 *  @param canId the id of the message you are sending
 *  @param txData the transmission data buffer
 *  @param txLen the length of data to be sent
 */
void transmit_can_data(CANDevice_t *device, const uint32_t canId,  const uint8_t txData[], const uint8_t txLen) {
	device->txHeader.DLC = txLen;
	device->txHeader.StdId = canId;
	device->txHeader.IDE = CAN_ID_STD;
	device->txHeader.RTR = CAN_RTR_DATA;

	if (HAL_CAN_AddTxMessage(device->hcan, &(device->txHeader), txData, device->txMailbox) != HAL_OK) {
		Error_Handler();
	}
}

