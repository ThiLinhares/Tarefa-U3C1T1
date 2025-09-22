/**
 * @file Sensor_Lumi.c
 * @brief Firmware para uma Luz Noturna Adaptativa com Raspberry Pi Pico.
 * @version 1.5 (Documentação Aprimorada)
 * @date 2025-09-22
 *
 * @details
 * Este projeto mede a intensidade da luz ambiente utilizando um sensor BH1750
 * e ajusta o brilho de um LED RGB (configurado como luz branca) através de PWM.
 * A lógica é projetada para um LED de Ânodo Comum, onde a intensidade do brilho
 * é inversamente proporcional ao duty cycle do PWM.
 *
 * Hardware:
 * - Placa: Raspberry Pi Pico W na BitDogLab
 * - Sensor: BH1750 (I2C)
 * - Atuador: LED RGB on-board (Ânodo Comum)
 */

// Inclusão de bibliotecas padrão e do SDK da Pico
#include <stdio.h>          // Necessário para a função printf (saída serial)
#include "pico/stdlib.h"     // Funções principais do SDK (gpio_*, sleep_ms, etc.)
#include "hardware/i2c.h"    // Funções para a comunicação I2C com o sensor
#include "hardware/pwm.h"    // Funções para o controle do LED via PWM

// --- Pinos e Constantes de Hardware ---
// Define qual barramento I2C será usado. A BitDogLab possui conectores para i2c0 e i2c1.
#define I2C_PORT i2c1
// Define os pinos de GPIO para o barramento I2C selecionado.
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// Define os pinos de GPIO conectados ao LED RGB na placa BitDogLab.
#define LED_R_PIN 13
#define LED_G_PIN 11
#define LED_B_PIN 12

// Constantes do sensor de luminosidade BH1750 (baseado no datasheet)
const uint8_t BH1750_ADDR = 0x23;                     // Endereço I2C padrão do sensor.
const uint8_t BH1750_CMD_POWER_ON = 0x01;             // Comando para ligar o sensor.
const uint8_t BH1750_CMD_CONTINUOUS_HIGH_RES = 0x10;  // Comando para medição contínua em alta resolução.

// --- Parâmetros de Calibração e Operação ---
/**
 * @brief Valor máximo de lux para a calibração.
 * @details Este é o principal parâmetro de ajuste. Acima deste valor de lux,
 * o LED terá brilho mínimo (quase apagado). Ajuste este valor para
 * corresponder à luminosidade máxima do seu ambiente de teste.
 */
const float LUX_MAX_RANGE = 1000.0f;

/**
 * @brief Nível máximo do contador PWM (resolução).
 * @details 65535 corresponde a uma resolução de 16-bit (2^16 - 1),
 * o que permite um controle de brilho muito mais suave do que uma resolução de 8-bit (255).
 */
const uint16_t PWM_MAX_LEVEL = 65535;

/**
 * @brief Mapeia um valor de luminosidade (lux) para um nível de PWM.
 * @param lux A leitura de luminosidade do sensor.
 * @return O valor do duty cycle (0-65535) a ser aplicado no PWM.
 */
uint16_t lux_to_pwm(float lux) {
    // Garante que o valor de entrada não saia da faixa esperada [0, LUX_MAX_RANGE].
    // Isso torna a função mais robusta contra leituras inesperadas do sensor.
    if (lux < 0) lux = 0;
    if (lux > LUX_MAX_RANGE) lux = LUX_MAX_RANGE;
    
    // Converte linearmente a faixa de lux [0, 1000] para a faixa de PWM [0, 65535].
    return (uint16_t)((lux / LUX_MAX_RANGE) * PWM_MAX_LEVEL);
}

/**
 * @brief Define o brilho do LED RGB, acionando todos os canais com o mesmo valor para gerar luz branca.
 * @param duty_cycle O nível de duty cycle (0-65535) a ser aplicado.
 */
void set_led_brightness(uint16_t duty_cycle) {
    // A função pwm_set_gpio_level do SDK cuida de encontrar o slice/canal correto para cada pino.
    pwm_set_gpio_level(LED_R_PIN, duty_cycle);
    pwm_set_gpio_level(LED_G_PIN, duty_cycle);
    pwm_set_gpio_level(LED_B_PIN, duty_cycle);
}

// --- Função Principal ---
int main() {
    // Inicializa a comunicação serial via USB, permitindo o uso do `printf` para depuração.
    stdio_init_all();
    sleep_ms(2000); // Pausa para dar tempo de conectar o monitor serial.
    printf(">> Luz Noturna Adaptativa (v1.5) <<\n");

    // --- Bloco de Configuração do I2C ---
    i2c_init(I2C_PORT, 100 * 1000); // Inicializa o barramento I2C com clock de 100kHz.
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Ativa os resistores de pull-up internos da Pico. Essencial para o funcionamento do I2C.
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // --- Bloco de Configuração do PWM ---
    const uint led_pins[] = {LED_R_PIN, LED_G_PIN, LED_B_PIN};
    for (int i = 0; i < 3; i++) {
        uint slice = pwm_gpio_to_slice_num(led_pins[i]);
        gpio_set_function(led_pins[i], GPIO_FUNC_PWM); // Informa ao pino que será controlado por PWM.
        pwm_set_wrap(slice, PWM_MAX_LEVEL);           // Define o período/resolução do PWM.
        pwm_set_enabled(slice, true);                 // Ativa o slice de PWM.
    }
    
    // --- Bloco de Configuração do Sensor BH1750 ---
    uint8_t cmd_on = BH1750_CMD_POWER_ON;
    uint8_t cmd_measure = BH1750_CMD_CONTINUOUS_HIGH_RES;
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, &cmd_on, 1, false);
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, &cmd_measure, 1, false);
    sleep_ms(120); // Pausa recomendada pelo datasheet para a primeira medição ficar pronta.

    // --- Loop Principal ---
    while (1) {
        uint8_t buffer[2];
        // Tenta ler 2 bytes do sensor. A função retorna o número de bytes lidos.
        if (i2c_read_blocking(I2C_PORT, BH1750_ADDR, buffer, 2, false) == 2) {
            // Combina os dois bytes (MSB e LSB) em um único valor de 16 bits.
            uint16_t raw_data = (buffer[0] << 8) | buffer[1];
            // Converte o valor bruto para lux usando o fator do datasheet.
            float lux = raw_data / 1.2f;

            // ** LÓGICA PRINCIPAL **
            // Para um LED de Ânodo Comum, o brilho é máximo quando o pino de controle está em GND (0V).
            // O duty cycle do PWM controla quanto tempo o sinal fica em VCC (3.3V).
            // Portanto, um duty cycle ALTO significa MENOS brilho.
            // A nossa conversão direta (lux -> duty_cycle) implementa corretamente este efeito inverso.
            uint16_t duty_cycle = lux_to_pwm(lux);
            set_led_brightness(duty_cycle);

            // Envia o status para o monitor serial para fins de depuração.
            printf("Luz: %.2f lux -> PWM: %d\n", lux, duty_cycle);
        } else {
             printf("Falha na leitura do sensor.\n");
        }
        // Pausa para evitar sobrecarga do monitor serial e criar uma frequência de atualização razoável.
        sleep_ms(100);
    }
    return 0; // Este ponto nunca será alcançado em um sistema embarcado.
}

