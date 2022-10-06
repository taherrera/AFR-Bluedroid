// Copyright 2018 Evandro Luis Copercini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _BLUETOOTH_SERIAL_H_
#define _BLUETOOTH_SERIAL_H_

#include "sdkconfig.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED) && defined(CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY)

#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>
//#include "BTScan.h"
#include <string.h>

typedef void (*BluetoothSerialDataCb)(const uint8_t *, size_t);
typedef void  (*ConfirmRequestCb)(uint32_t);
typedef void (*AuthCompleteCb)(bool);
//typedef void (*BTAdvertisedDeviceCb)(BTAdvertisedDevice*);


bool BluetoothSerial_begin(char * localName, bool isMaster);
int BluetoothSerial_available(void);
int BluetoothSerial_peek(void);
bool BluetoothSerial_hasClient(void);
int BluetoothSerial_read(void);
size_t BluetoothSerial_write(const uint8_t *buffer, size_t size);
void BluetoothSerial_flush(void);
void BluetoothSerial_end(void);
void BluetoothSerial_onData(BluetoothSerialDataCb cb);
esp_err_t BluetoothSerial_register_callback(esp_spp_cb_t * callback);

void BluetoothSerial_onConfirmRequest(ConfirmRequestCb cb);
void BluetoothSerial_onAuthComplete(AuthCompleteCb cb);
void BluetoothSerial_confirmReply(bool confirm);

void BluetoothSerial_enableSSP();
bool BluetoothSerial_setPin(const char *pin);
bool BluetoothSerial_connect_name(char * remoteName);
bool BluetoothSerial_connect_addr(uint8_t remoteAddress[]);
bool BluetoothSerial_connect();
bool BluetoothSerial_connected(int timeout/*=0*/);
bool BluetoothSerial_isReady(bool checkMaster/*=false*/, int timeout/*=0*/);
bool BluetoothSerial_disconnect();
bool BluetoothSerial_unpairDevice(uint8_t remoteAddress[]);



char * BluetoothSerial_local_name;

#else
#error You must enable bluetooth classic in menuconfig and Bluetooth controler mode (BR/EDR Only)

#endif

#endif
