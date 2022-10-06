/* 
  author: Tomas Herrera taherrera@uc.cl
*/

#include "vfs.h"
#include <esp_vfs_fat.h>

int vfs_init(void){
  int ret;

  const esp_vfs_fat_mount_config_t mount_config = { .format_if_mount_failed = false,
                                                    .max_files = 5,
                                                    .allocation_unit_size = 16 * 1024};

  sdmmc_host_t host_config_input = SDSPI_HOST_DEFAULT();
  host_config_input.slot = SPI2_HOST;
  host_config_input.max_freq_khz = 4000;//SDMMC_FREQ_PROBING;
  //host_config_input.command_timeout_ms = 0;
  //host_config_input.init();
  
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = SPI2_HOST;
  slot_config.gpio_cs = SDSPI_SLOT_NO_CD;
  slot_config.gpio_cd= SDSPI_SLOT_NO_CD;
  slot_config.gpio_wp = SDSPI_SLOT_NO_WP;
  slot_config.gpio_int = SDSPI_SLOT_NO_INT;



  sdmmc_card_t sdmmc_card = {.host = host_config_input}; 

  ret = esp_vfs_fat_sdspi_mount("/sdcard", 
                                    &host_config_input,
                                    &slot_config,
                                    &mount_config,
                                    &sdmmc_card);

  if (ret != ESP_OK){
    printf("[E] vfs.c vfs_init: Error %s (%d) mounting card\n",esp_err_to_name(ret) , ret);
    return -2;
  }

  sdmmc_card_print_info(stdout, sdmmc_card);

  return ret;
}

int test_sd(void){

  FILE *f = fopen("/sdcard/test.txt", "w");
  if (f == NULL){
    printf("Error creating file\n");
    return -1;
  }
  fprintf(f, "Hello %s!\n", "Diego");
  fclose(f);  
  return 0;
}


