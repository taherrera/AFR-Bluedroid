/* 
  author: Tomas Herrera taherrera@uc.cl
*/

#include "FreeRTOS.h"
#include "task.h"

#include "nvs_flash.h"
#include "nvs.h"


/* program entry */
int app_main(void){

  nvs_flash_init(); 
  bluetooth_init("test2");
  vTaskStartScheduler();

  while(1){
    vTaskDelay(100);
  }


  return 0;
}

/* Weas que necesita amazon-freertos */

extern void esp_vApplicationTickHook();
void IRAM_ATTR vApplicationTickHook(){
    esp_vApplicationTickHook();
}

extern void esp_vApplicationIdleHook();
void vApplicationIdleHook(){
    esp_vApplicationIdleHook();
}

void vApplicationDaemonTaskStartupHook( void ){
}
