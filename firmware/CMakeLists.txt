cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nfc_ble_auth)

target_sources(app PRIVATE
    src/main.c
    src/nfc_handler.c
    src/ble_manager.c
    src/gatt_service.c
    src/power_manager.c
    src/ad5940.c
    src/ad5940_port.c
    src/RampTest.c
    src/CV.c
    src/ad5940_lp.c
    src/CA.c
    src/DPV.c
)
target_include_directories(app PRIVATE src)
# CHIPSEL_594X sélectionne les chemins de code AD5940 (pas ADuCM355)
target_compile_definitions(app PRIVATE CHIPSEL_594X)