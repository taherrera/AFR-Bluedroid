/*
 * BTScanResultsSet.cpp
 *
 *  Created on: Feb 5, 2021
 *      Author: Thomas M. (ArcticSnowSky)
 */

#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)


#include <esp_err.h>

#include "BTAdvertisedDevice.h"
#include "BTScan.h"
//#include "GeneralUtils.h"
#include "esp32-hal-log.h"


class BTAdvertisedDevice;



/**
 * @brief Return the count of devices found in the last scan.
 * @return The number of devices found in the last scan.
 */
int BTScanResultsSet::getCount() {
	return m_vectorAdvertisedDevices.size();
} // getCount


/**
 * @brief Return the specified device at the given index.
 * The index should be between 0 and getCount()-1.
 * @param [in] i The index of the device.
 * @return The device at the specified index.
 */
BTAdvertisedDevice* BTScanResultsSet::getDevice(uint32_t i) {
	if (i < 0)
		return nullptr;

	uint32_t x = 0;
	BTAdvertisedDeviceSet* pDev = &m_vectorAdvertisedDevices.begin()->second;
	for (auto it = m_vectorAdvertisedDevices.begin(); it != m_vectorAdvertisedDevices.end(); it++) {
		pDev = &it->second;
		if (x==i)	break;
		x++;
	}
	return x==i ? pDev : nullptr;
}

void BTScanResultsSet::clear() {
    //for(auto _dev : m_vectorAdvertisedDevices)
	//	delete _dev.second;
	m_vectorAdvertisedDevices.clear();
}

bool BTScanResultsSet::add(BTAdvertisedDeviceSet advertisedDevice, bool unique) {
	std::string key = advertisedDevice.getAddress().toString();
	if (!unique || m_vectorAdvertisedDevices.count(key) == 0) {
		m_vectorAdvertisedDevices.insert(std::pair<std::string, BTAdvertisedDeviceSet>(key, advertisedDevice));
		return true;
	} else
		return false;
}

#endif
