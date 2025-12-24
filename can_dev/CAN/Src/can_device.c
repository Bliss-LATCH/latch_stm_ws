/*
 * can_device.c
 *
 *  Created on: Dec 23, 2025
 *      Author: nick
 */

#include "can_device.h"

/** @breif Configure what ids we can to look for on the CAN bus
 *  @param hcan can handle to configure
 * 	@param idList list of ids that
 */
void config_device_filter(CAN_HandleTypeDef *hcan, uint16_t *idList, uint16_t idCount) {
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

		uint16_t shifted_id = *(idList+i)<<5; // left shifted by 5 since thats where the id bits start

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
			HAL_CAN_ConfigFilter(hcan, &filter);

			filter.FilterIdHigh = 0;
			filter.FilterIdLow = 0;
			filter.FilterMaskIdHigh = 0;
			filter.FilterMaskIdLow = 0;
		}
	}

};
