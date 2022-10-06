vendors/espressif/esp-idf/install.sh

To compile and run:

export IDF_PATH=/home/tom/Documents/SLT/tecnocal/instacrop/fw/instacrop_test/amazon-freertos/vendors/espressif/esp-idf
source /home/tom/Documents/SLT/tecnocal/instacrop/fw/instacrop_test/amazon-freertos/vendors/espressif/esp-idf/export.sh
export ESPBAUD=1000000

idf.py -DVENDOR=espressif -DBOARD=esp32_wrover_kit -DCOMPILER=xtensa-esp32 -DCMAKE_TOOLCHAIN_FILE=amazon-freertos/tools/cmake/toolchains/xtensa-esp32.cmake set-target esp32
idf.py -DVENDOR=espressif -DBOARD=esp32_wrover_kit -DCOMPILER=xtensa-esp32 -DCMAKE_TOOLCHAIN_FILE=amazon-freertos/tools/cmake/toolchains/xtensa-esp32.cmake menuconfig
idf.py -DVENDOR=espressif -DBOARD=esp32_wrover_kit -DCOMPILER=xtensa-esp32 -DCMAKE_TOOLCHAIN_FILE=amazon-freertos/tools/cmake/toolchains/xtensa-esp32.cmake build
idf.py -DVENDOR=espressif -DBOARD=esp32_wrover_kit -DCOMPILER=xtensa-esp32 -DCMAKE_TOOLCHAIN_FILE=amazon-freertos/tools/cmake/toolchains/xtensa-esp32.cmake -p /dev/ttyUSB0 flash
python3 amazon-freertos/vendors/espressif/esp-idf/tools/idf_monitor.py --port /dev/ttyUSB0 --baud 115200 build/instacrop_app

