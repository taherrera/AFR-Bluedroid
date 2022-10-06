// Copyright 2018 Evandro Luis Copercini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"


#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)


#include "BluetoothSerial.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include <esp_log.h>


const char * _spp_server_name = "ESP32SPP";

#define RX_QUEUE_SIZE 512
#define TX_QUEUE_SIZE 32
#define SPP_TX_QUEUE_TIMEOUT 1000
#define SPP_TX_DONE_TIMEOUT 1000
#define SPP_CONGESTED_TIMEOUT 1000

static uint32_t _spp_client = 0;
static xQueueHandle _spp_rx_queue = NULL;
static xQueueHandle _spp_tx_queue = NULL;
static SemaphoreHandle_t _spp_tx_done = NULL;
static TaskHandle_t _spp_task_handle = NULL;
static EventGroupHandle_t _spp_event_group = NULL;
static EventGroupHandle_t _bt_event_group = NULL;
static bool secondConnectionAttempt;
static esp_spp_cb_t * custom_spp_callback = NULL;
static BluetoothSerialDataCb custom_data_callback = NULL;
static esp_bd_addr_t current_bd_addr;
static ConfirmRequestCb confirm_request_callback = NULL;
static AuthCompleteCb auth_complete_callback = NULL;

#define INQ_LEN 0x10
#define INQ_NUM_RSPS 20
#define READY_TIMEOUT (10 * 1000)
#define SCAN_TIMEOUT (INQ_LEN * 2 * 1000)
static esp_bd_addr_t _peer_bd_addr;
static char _remote_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static bool _isRemoteAddressSet;
static bool _isMaster;
static esp_bt_pin_code_t _pin_code;
static int _pin_len;
static bool _isPinSet;
static bool _enableSSP;

#define SPP_RUNNING     0x01
#define SPP_CONNECTED   0x02
#define SPP_CONGESTED   0x04
#define SPP_DISCONNECTED 0x08

#define BT_DISCOVERY_RUNNING    0x01
#define BT_DISCOVERY_COMPLETED  0x02

const int INQ_TIME = 1280;   // Inquire Time unit 1280 ms
const int MIN_INQ_TIME = (ESP_BT_GAP_MIN_INQ_LEN * INQ_TIME);
const int MAX_INQ_TIME = (ESP_BT_GAP_MAX_INQ_LEN * INQ_TIME);

#include "esp_bt.h"

#ifdef CONFIG_BTDM_CONTROLLER_MODE_BTDM
#define BT_MODE ESP_BT_MODE_BTDM
#elif defined(CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY)
#define BT_MODE ESP_BT_MODE_CLASSIC_BT
#else
#define BT_MODE ESP_BT_MODE_BLE
#endif

typedef struct {
        size_t len;
        uint8_t data[];
} spp_packet_t;

#if (ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO)
static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }

  uint8_t *p = bda;
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          p[0], p[1], p[2], p[3], p[4], p[5]);
  return str;
}
#endif

static bool btStarted(void){
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
}

static bool btStart(void){
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        return true;
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){
        esp_bt_controller_init(&cfg);
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){}
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        if (esp_bt_controller_enable(BT_MODE)) {
            return false;
        }
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        return true;
    }
    return false;
}

static bool btStop(){
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){
        return true;
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        if (esp_bt_controller_disable()) {
            return false;
        }
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        if (esp_bt_controller_deinit()) {
			return false;
		}
		vTaskDelay(1);
		if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {			
			return false;		
		}
        return true;
    }
    return false;
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    if (!eir || !bdname || !bdname_len) {
        return false;
    }

    uint8_t *rmt_bdname, rmt_bdname_len;
    *bdname = *bdname_len = rmt_bdname_len = 0;

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }
    if (rmt_bdname) {
        rmt_bdname_len = rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN ? ESP_BT_GAP_MAX_BDNAME_LEN : rmt_bdname_len;
        memcpy(bdname, rmt_bdname, rmt_bdname_len);
        bdname[rmt_bdname_len] = 0;
        *bdname_len = rmt_bdname_len;
        return true;
    }
    return false;
}

static bool btSetPin() {
    esp_bt_pin_type_t pin_type;
    if (_isPinSet) {
        if (_pin_len) {
            pin_type = ESP_BT_PIN_TYPE_FIXED;
        } else {
            _isPinSet = false;
            pin_type = ESP_BT_PIN_TYPE_VARIABLE; // pin_code would be ignored (default)
        }
        return (esp_bt_gap_set_pin(pin_type, _pin_len, _pin_code) == ESP_OK);        
    }
    return false;
}

static esp_err_t _spp_queue_packet(uint8_t *data, size_t len){
    if(!data || !len){
        return ESP_OK;
    }
    spp_packet_t * packet = (spp_packet_t*)malloc(sizeof(spp_packet_t) + len);
    if(!packet){
        return ESP_FAIL;
    }
    packet->len = len;
    memcpy(packet->data, data, len);
    if (!_spp_tx_queue || xQueueSend(_spp_tx_queue, &packet, SPP_TX_QUEUE_TIMEOUT) != pdPASS) {
        free(packet);
        return ESP_FAIL;
    }
    return ESP_OK;
}

#define SPP_TX_MAX 330
static uint8_t _spp_tx_buffer[SPP_TX_MAX];
static uint16_t _spp_tx_buffer_len = 0;

static bool _spp_send_buffer(){
    if((xEventGroupWaitBits(_spp_event_group, SPP_CONGESTED, pdFALSE, pdTRUE, SPP_CONGESTED_TIMEOUT) & SPP_CONGESTED) != 0){
        if(!_spp_client){
            return false;
        }
        esp_err_t err = esp_spp_write(_spp_client, _spp_tx_buffer_len, _spp_tx_buffer);
        if(err != ESP_OK){
            return false;
        }
        _spp_tx_buffer_len = 0;
        if(xSemaphoreTake(_spp_tx_done, SPP_TX_DONE_TIMEOUT) != pdTRUE){
            return false;
        }
        return true;
    }
    return false;
}

static void _spp_tx_task(void * arg){
    //esp_task_wdt_add(NULL); //add current thread to WDT watch
    spp_packet_t *packet = NULL;
    size_t len = 0, to_send = 0;
    uint8_t * data = NULL;
    for (;;) {
        //esp_task_wdt_reset();
        if(_spp_tx_queue && xQueueReceive(_spp_tx_queue, &packet, portMAX_DELAY) == pdTRUE && packet){
            if(packet->len <= (SPP_TX_MAX - _spp_tx_buffer_len)){
                memcpy(_spp_tx_buffer+_spp_tx_buffer_len, packet->data, packet->len);
                _spp_tx_buffer_len+=packet->len;
                free(packet);
                packet = NULL;
                if(SPP_TX_MAX == _spp_tx_buffer_len || uxQueueMessagesWaiting(_spp_tx_queue) == 0){
                    _spp_send_buffer();
                }
            } else {
                len = packet->len;
                data = packet->data;
                to_send = SPP_TX_MAX - _spp_tx_buffer_len;
                memcpy(_spp_tx_buffer+_spp_tx_buffer_len, data, to_send);
                _spp_tx_buffer_len = SPP_TX_MAX;
                data += to_send;
                len -= to_send;
                if(!_spp_send_buffer()){
                    len = 0;
                }
                while(len >= SPP_TX_MAX){
                    memcpy(_spp_tx_buffer, data, SPP_TX_MAX);
                    _spp_tx_buffer_len = SPP_TX_MAX;
                    data += SPP_TX_MAX;
                    len -= SPP_TX_MAX;
                    if(!_spp_send_buffer()){
                        len = 0;
                        break;
                    }
                }
                if(len){
                    memcpy(_spp_tx_buffer, data, len);
                    _spp_tx_buffer_len += len;
                    if(uxQueueMessagesWaiting(_spp_tx_queue) == 0){
                        _spp_send_buffer();
                    }
                }
                free(packet);
                packet = NULL;
            }
        } else { /* something whent wrong */
        }
    }
    vTaskDelete(NULL);
    _spp_task_handle = NULL;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
#ifdef ESP_IDF_VERSION_MAJOR
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif
        if (!_isMaster) {

            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, _spp_server_name);
        }
        xEventGroupSetBits(_spp_event_group, SPP_RUNNING);
        break;

    case ESP_SPP_SRV_OPEN_EVT://Server connection open
        if (param->srv_open.status == ESP_SPP_SUCCESS) {

            if (!_spp_client){
                _spp_client = param->srv_open.handle;
                _spp_tx_buffer_len = 0;
            } else {
                secondConnectionAttempt = true;
                esp_spp_disconnect(param->srv_open.handle);
            }
            xEventGroupClearBits(_spp_event_group, SPP_DISCONNECTED);
            xEventGroupSetBits(_spp_event_group, SPP_CONNECTED);
        } else {

        }
        break;

    case ESP_SPP_CLOSE_EVT://Client connection closed
        if ((param->close.async == false && param->close.status == ESP_SPP_SUCCESS) || param->close.async) {

            if(secondConnectionAttempt) {
                secondConnectionAttempt = false;
            } else {
                _spp_client = 0;
                xEventGroupSetBits(_spp_event_group, SPP_DISCONNECTED);
                xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
            }        
            xEventGroupClearBits(_spp_event_group, SPP_CONNECTED);
        } else {
            /* fail */
        }
        break;

    case ESP_SPP_CONG_EVT://connection congestion status changed
        if(param->cong.cong){
            xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
        } else {
            xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        }
        break;

    case ESP_SPP_WRITE_EVT://write operation completed
        if (param->write.status == ESP_SPP_SUCCESS) {
            if(param->write.cong){
                xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
            }
        } else {
            /* fail */
        }
        xSemaphoreGive(_spp_tx_done);//we can try to send another packet
        break;

    case ESP_SPP_DATA_IND_EVT://connection received data
        //esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len); //for low level debug
        //ets_printf("r:%u\n", param->data_ind.len);

        if(custom_data_callback){
            custom_data_callback(param->data_ind.data, param->data_ind.len);
        } else if (_spp_rx_queue != NULL){
            for (int i = 0; i < param->data_ind.len; i++){
                if(xQueueSend(_spp_rx_queue, param->data_ind.data + i, (TickType_t)0) != pdTRUE){
                    break;
                }
            }
        }
        break;

    case ESP_SPP_DISCOVERY_COMP_EVT://discovery complete
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, param->disc_comp.scn[0], _peer_bd_addr);
        } else {
        }
        break;

    case ESP_SPP_OPEN_EVT://Client connection open
        if (!_spp_client){
                _spp_client = param->open.handle;
        } else {
            secondConnectionAttempt = true;
            esp_spp_disconnect(param->open.handle);
        }
        xEventGroupClearBits(_spp_event_group, SPP_DISCONNECTED);
        xEventGroupSetBits(_spp_event_group, SPP_CONNECTED);
        xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        break;

    case ESP_SPP_START_EVT://server started
        break;

    case ESP_SPP_CL_INIT_EVT://client initiated a connection
        break;

    default:
        break;
    }
    if(custom_spp_callback)(*custom_spp_callback)(event, param);
}

void BluetoothSerial_onData(BluetoothSerialDataCb cb){
    custom_data_callback = cb;
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
        case ESP_BT_GAP_DISC_RES_EVT:

            break;

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                xEventGroupClearBits(_bt_event_group, BT_DISCOVERY_RUNNING);
                xEventGroupSetBits(_bt_event_group, BT_DISCOVERY_COMPLETED);
            } else { // ESP_BT_GAP_DISCOVERY_STARTED
                xEventGroupClearBits(_bt_event_group, BT_DISCOVERY_COMPLETED);
                xEventGroupSetBits(_bt_event_group, BT_DISCOVERY_RUNNING);
            }
            break;

        case ESP_BT_GAP_RMT_SRVCS_EVT:

            break;

        case ESP_BT_GAP_RMT_SRVC_REC_EVT:

            break;

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {

                if (auth_complete_callback) {
                    auth_complete_callback(true);
                }
            } else {

                if (auth_complete_callback) {
                    auth_complete_callback(false);
                }
            }
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            // default pairing pins

            if (param->pin_req.min_16_digit) {

                esp_bt_pin_code_t pin_code;
                memset(pin_code, '0', ESP_BT_PIN_CODE_LEN);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {

                esp_bt_pin_code_t pin_code;
                memcpy(pin_code, "1234", 4);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
       
        case ESP_BT_GAP_CFM_REQ_EVT:

            if (confirm_request_callback) {
                memcpy(current_bd_addr, param->cfm_req.bda, sizeof(esp_bd_addr_t));
                confirm_request_callback(param->cfm_req.num_val);
            }
            else {
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            }
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT:

            break;

        case ESP_BT_GAP_KEY_REQ_EVT:

            break;

        default:
            break;
    }
}

static bool _init_bt(const char *deviceName)
{
    if(!_bt_event_group){
        _bt_event_group = xEventGroupCreate();
        if(!_bt_event_group){

            return false;
        }
        xEventGroupClearBits(_bt_event_group, 0xFFFFFF);
    }
    if(!_spp_event_group){
        _spp_event_group = xEventGroupCreate();
        if(!_spp_event_group){

            return false;
        }
        xEventGroupClearBits(_spp_event_group, 0xFFFFFF);
        xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
        xEventGroupSetBits(_spp_event_group, SPP_DISCONNECTED);
    }
    if (_spp_rx_queue == NULL){
        _spp_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(uint8_t)); //initialize the queue
        if (_spp_rx_queue == NULL){

            return false;
        }
    }
    if (_spp_tx_queue == NULL){
        _spp_tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(spp_packet_t*)); //initialize the queue
        if (_spp_tx_queue == NULL){

            return false;
        }
    }
    if(_spp_tx_done == NULL){
        _spp_tx_done = xSemaphoreCreateBinary();
        if (_spp_tx_done == NULL){

            return false;
        }
        xSemaphoreTake(_spp_tx_done, 0);
    }

    if(!_spp_task_handle){
        xTaskCreatePinnedToCore(_spp_tx_task, "spp_tx", 4096, NULL, 10, &_spp_task_handle, 0 /* CPU */);
        if(!_spp_task_handle){

            return false;
        }
    }

    if (!btStarted() && !btStart()){

        return false;
    }
    
    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
    if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        if (esp_bluedroid_init()) {

            return false;
        }
    }
    
    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED){
        if (esp_bluedroid_enable()) {

            return false;
        }
    }

    // Why only master need this?  Slave need this during pairing as well
//    if (_isMaster && esp_bt_gap_register_callback(esp_bt_gap_cb) != ESP_OK) {
    if (esp_bt_gap_register_callback(esp_bt_gap_cb) != ESP_OK) {

        return false;
    }

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK){

        return false;
    }

    if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK){

        return false;
    }

    // if (esp_bt_sleep_disable() != ESP_OK){
    //     log_e("esp_bt_sleep_disable failed");
    // }


    esp_bt_dev_set_device_name(deviceName);

    if (_isPinSet) {
        btSetPin();
    }

    if (_enableSSP) {

        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    }

    // the default BTA_DM_COD_LOUDSPEAKER does not work with the macOS BT stack
    esp_bt_cod_t cod;
    cod.major = 0b00001;
    cod.minor = 0b000100;
    cod.service = 0b00000010110;
    if (esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD) != ESP_OK) {

        return false;
    }
    return true;
}

static bool _stop_bt()
{
    if (btStarted()){
        if(_spp_client)
            esp_spp_disconnect(_spp_client);
        esp_spp_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        btStop();
    }
    _spp_client = 0;
    if(_spp_task_handle){
        vTaskDelete(_spp_task_handle);
        _spp_task_handle = NULL;
    }
    if(_spp_event_group){
        vEventGroupDelete(_spp_event_group);
        _spp_event_group = NULL;
    }
    if(_spp_rx_queue){
        vQueueDelete(_spp_rx_queue);
        //ToDo: clear RX queue when in packet mode
        _spp_rx_queue = NULL;
    }
    if(_spp_tx_queue){
        spp_packet_t *packet = NULL;
        while(xQueueReceive(_spp_tx_queue, &packet, 0) == pdTRUE){
            free(packet);
        }
        vQueueDelete(_spp_tx_queue);
        _spp_tx_queue = NULL;
    }
    if (_spp_tx_done) {
        vSemaphoreDelete(_spp_tx_done);
        _spp_tx_done = NULL;
    }
    if (_bt_event_group) {
        vEventGroupDelete(_bt_event_group);
        _bt_event_group = NULL;
    }
    return true;
}

static bool waitForConnect(int timeout) {
    TickType_t xTicksToWait = timeout / portTICK_PERIOD_MS;
    return (xEventGroupWaitBits(_spp_event_group, SPP_CONNECTED, pdFALSE, pdTRUE, xTicksToWait) & SPP_CONNECTED) != 0;
}

static bool waitForDiscovered(int timeout) {
    TickType_t xTicksToWait = timeout / portTICK_PERIOD_MS;
    return (xEventGroupWaitBits(_spp_event_group, BT_DISCOVERY_COMPLETED, pdFALSE, pdTRUE, xTicksToWait) & BT_DISCOVERY_COMPLETED) != 0;
}


bool BluetoothSerial_begin(char * local_name, bool isMaster)
{
    _isMaster = isMaster;
    return _init_bt(local_name);
}

int BluetoothSerial_available(void)
{
    if (_spp_rx_queue == NULL){
        return 0;
    }
    return uxQueueMessagesWaiting(_spp_rx_queue);
}

int BluetoothSerial_peek(void)
{
    uint8_t c;
    if (_spp_rx_queue && xQueuePeek(_spp_rx_queue, &c, 0)){
        return c;
    }
    return -1;
}

bool BluetoothSerial_hasClient(void)
{
    return _spp_client > 0;
}

int BluetoothSerial_read(void)
{

    uint8_t c = 0;
    if (_spp_rx_queue && xQueueReceive(_spp_rx_queue, &c, 0)){
        return c;
    }
    return -1;
}

size_t BluetoothSerial_write(const uint8_t *buffer, size_t size)
{
    if (!_spp_client){
        return 0;
    }
    return (_spp_queue_packet((uint8_t *)buffer, size) == ESP_OK) ? size : 0;
}

void BluetoothSerial_flush()
{
    if (_spp_tx_queue != NULL){
        while(uxQueueMessagesWaiting(_spp_tx_queue) > 0){
           vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void BluetoothSerial_end()
{
    _stop_bt();
}

void BluetoothSerial_onConfirmRequest(ConfirmRequestCb cb)
{
    confirm_request_callback = cb;
}

void BluetoothSerial_onAuthComplete(AuthCompleteCb cb)
{
    auth_complete_callback = cb;
}

void BluetoothSerial_confirmReply(bool confirm)
{
    esp_bt_gap_ssp_confirm_reply(current_bd_addr, confirm);  
}


esp_err_t BluetoothSerial_register_callback(esp_spp_cb_t * callback)
{
    custom_spp_callback = callback;
    return ESP_OK;
}

//Simple Secure Pairing
void BluetoothSerial_enableSSP() {
    _enableSSP = true;
}
/*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
*/
bool BluetoothSerial_setPin(const char *pin) {
    bool isEmpty =  !(pin  && *pin);
    if (isEmpty && !_isPinSet) {
        return true; // nothing to do
    } else if (!isEmpty){
        _pin_len = strlen(pin);
        memcpy(_pin_code, pin, _pin_len);
    } else {
        _pin_len = 0; // resetting pin to none (default)
    }
    _pin_code[_pin_len] = 0;
    _isPinSet = true;
    if (BluetoothSerial_isReady(false, READY_TIMEOUT)) {
        btSetPin();
    }
    return true;
}

bool BluetoothSerial_connect_name(char * remoteName)
{
    if (!BluetoothSerial_isReady(true, READY_TIMEOUT)) return false;
    if (remoteName && strlen(remoteName) < 1) {

        return false; 
    }
    BluetoothSerial_disconnect();
    _isRemoteAddressSet = false;
    strncpy(_remote_name, remoteName, ESP_BT_GAP_MAX_BDNAME_LEN);
    _remote_name[ESP_BT_GAP_MAX_BDNAME_LEN] = 0;

    // will first resolve name to address
#ifdef ESP_IDF_VERSION_MAJOR
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif
    if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQ_LEN, INQ_NUM_RSPS) == ESP_OK) {
        return waitForConnect(SCAN_TIMEOUT);
    }
    return false;
}

bool BluetoothSerial_connect_addr(uint8_t remoteAddress[])
{
    if (!BluetoothSerial_isReady(true, READY_TIMEOUT)) return false;
    if (!remoteAddress) {

        return false; 
    }
    BluetoothSerial_disconnect();
    _remote_name[0] = 0;
    _isRemoteAddressSet = true;
    memcpy(_peer_bd_addr, remoteAddress, ESP_BD_ADDR_LEN);

    if (esp_spp_start_discovery(_peer_bd_addr) == ESP_OK) {
        return waitForConnect(READY_TIMEOUT);
    }
    return false;
}

bool BluetoothSerial_connect()
{
    if (!BluetoothSerial_isReady(true, READY_TIMEOUT)) return false;
    if (_isRemoteAddressSet){
        BluetoothSerial_disconnect();
        // use resolved or set address first

        if (esp_spp_start_discovery(_peer_bd_addr) == ESP_OK) {
            return waitForConnect(READY_TIMEOUT);
        }
        return false;
    } else if (_remote_name[0]) {
        BluetoothSerial_disconnect();

        // will resolve name to address first - it may take a while
#ifdef ESP_IDF_VERSION_MAJOR
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif
        if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, INQ_LEN, INQ_NUM_RSPS) == ESP_OK) {
            return waitForConnect(SCAN_TIMEOUT);
        }
        return false;
    }

    return false;
}

bool BluetoothSerial_disconnect() {
    if (_spp_client) {
        BluetoothSerial_flush();

        if (esp_spp_disconnect(_spp_client) == ESP_OK) {
            TickType_t xTicksToWait = READY_TIMEOUT / portTICK_PERIOD_MS;
            return (xEventGroupWaitBits(_spp_event_group, SPP_DISCONNECTED, pdFALSE, pdTRUE, xTicksToWait) & SPP_DISCONNECTED) != 0;
        }
    }
    return false;
}

bool BluetoothSerial_unpairDevice(uint8_t remoteAddress[]) {
    if (BluetoothSerial_isReady(false, READY_TIMEOUT)) {

        return (esp_bt_gap_remove_bond_device(remoteAddress) == ESP_OK);
    }
    return false;
}

bool BluetoothSerial_connected(int timeout) {
    return waitForConnect(timeout);
}

bool BluetoothSerial_isReady(bool checkMaster, int timeout) {
    if (checkMaster && !_isMaster) {

        return false;
    }
    if (!btStarted()) {

        return false;
    }
    TickType_t xTicksToWait = timeout / portTICK_PERIOD_MS;
    return (xEventGroupWaitBits(_spp_event_group, SPP_RUNNING, pdFALSE, pdTRUE, xTicksToWait) & SPP_RUNNING) != 0;
}

#else

#error "must define CONFIG_BT_ENABLED & CONFIG_BLUEDROID_ENABLED"

#endif // if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
