# Projeto: Luz Noturna Adaptativa com Pico W

Este projeto implementa um sistema de iluminação inteligente que ajusta a intensidade de um LED branco com base na luminosidade do ambiente, utilizando um microcontrolador Raspberry Pi Pico W em uma placa BitDogLab.

## Descrição do Funcionamento

O sistema utiliza um sensor de luminosidade BH1750 para medir continuamente a luz ambiente. Com base nessa leitura, a intensidade de um LED RGB (configurado para emitir luz branca) é controlada via PWM (Pulse Width Modulation).

A lógica é inversa:
-   **Ambiente claro:** A leitura de lux é alta, e o LED fica apagado ou com brilho mínimo.
-   **Ambiente escuro:** A leitura de lux é baixa, e o LED acende com intensidade máxima.

Isso cria um efeito de "luz de presença" ou "luz noturna" que só acende quando necessário, economizando energia e proporcionando conforto visual.

## Hardware Necessário

-   Raspberry Pi Pico W
-   Placa BitDogLab (ou protoboard e jumpers)
-   Sensor de Luminosidade BH1750 (módulo com pinos)
-   LED RGB Embutido na placa BitDogLab
-   Cabo Micro-USB

## Esquema de Conexão

O sensor BH1750 deve ser conectado a um dos conectores I2C da placa BitDogLab. O código está configurado para usar o **`I2C1`**.

-   **Sensor BH1750 VCC** -> **Pino 3.3V** da BitDogLab
-   **Sensor BH1750 GND** -> **Pino GND** da BitDogLab
-   **Sensor BH1750 SDA** -> **Pino SDA do conector I2C1 (GPIO2)** da BitDogLab
-   **Sensor BH1750 SCL** -> **Pino SCL do conector I2C1 (GPIO3)** da BitDogLab

O LED RGB já está integrado na placa e conectado aos seguintes pinos:
-   **LED Vermelho:** `GPIO13`
-   **LED Verde:** `GPIO11`
-   **LED Azul:** `GPIO12`

## Software e Ferramentas

-   **IDE:** Visual Studio Code
-   **Compilador:** ARM GCC Compiler
-   **SDK:** Raspberry Pi Pico SDK
-   **Ferramentas de Build:** CMake e Ninja

## Como Compilar e Executar

1.  **Clone o repositório** para o seu computador.
2.  **Abra a pasta do projeto** no Visual Studio Code.
3.  **Configure o projeto:** Se for a primeira vez, use a paleta de comandos (`Ctrl+Shift+P`) e execute `CMake: Configure`. Selecione o compilador `GCC for arm-none-eabi`.
4.  **Compile o projeto:** Clique no botão **Build** na barra de status do VS Code ou execute o comando `CMake: Build`.
5.  **Grave na Pico:**
    -   Conecte a Pico ao computador segurando o botão `BOOTSEL`.
    -   A Pico aparecerá como um dispositivo de armazenamento USB.
    -   Arraste o arquivo `Sensor_Lumi.uf2` (localizado na pasta `build`) para dentro da Pico.
6.  **Monitore a Saída:** Abra um monitor serial (como o Serial Monitor do VS Code ou PuTTY) para ver as leituras de lux e o nível de brilho do LED.

## Autor

-   [Seu Nome Aqui]
