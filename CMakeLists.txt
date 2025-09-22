# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Versão mínima do CMake
cmake_minimum_required(VERSION 3.13)

# Importa as configurações do Pico SDK
include(pico_sdk_import.cmake)

# Define o nome do projeto
project(Sensor_Lumi C CXX ASM)

# Inicializa o SDK
pico_sdk_init()

# Define o nome do nosso executável.
# ATENÇÃO: Lembre-se de renomear seu arquivo .c para "Sensor_Lumi.c"
# ou altere a linha abaixo para o nome que você está usando.
add_executable(Sensor_Lumi
    Sensor_Lumi.c
)

# Linka as bibliotecas necessárias do SDK
# Precisamos da 'pico_stdlib' (padrão) e da 'hardware_i2c'
target_link_libraries(Sensor_Lumi pico_stdlib hardware_i2c hardware_pwm)

# Habilita a saída USB (para o printf) e UART
pico_enable_stdio_usb(Sensor_Lumi 1)
pico_enable_stdio_uart(Sensor_Lumi 1)

# Gera os arquivos .uf2, .elf, etc.
pico_add_extra_outputs(Sensor_Lumi)