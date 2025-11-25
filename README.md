# Projeto Final de Sistemas Embarcados

## Descrição Geral

Este projeto consiste no desenvolvimento de uma fonte ajustável baseada em um conversor **buck** controlado por **PWM**. O sistema deve regular a tensão de saída para atingir um valor de referência, além de medir e analisar os sinais de entrada e saída.

## Funcionalidades

- **Conversor Buck** como base do sistema.
- **Controle PID** implementado em *hard time*.
- **Três botões físicos**:
  - Dois botões para ajustar a amplitude (aumentar e diminuir).
  - Um botão para selecionar o formato do sinal de saída *(A frequência permanece constante entre todos os formatos.)*:
    - CC
    - Senoidal
    - Triangular
    - Quadrada  
- **Display LCD** mostrando:
  - Amplitude da referência
  - Tensão de entrada
  - Tensão de saída
- **Watchdog de software** para segurança do sistema.

## Objetivo

Controlar o conversor buck para alcançar a tensão de referência utilizando PWM, realizar a leitura das tensões de entrada e saída, calcular suas FFT e enviar todos os dados ao computador via UART.

## Observações de Implementação
- Utilizar inicialmente ponto flutuante para desenvolvimento e teste.
- Converter as operações para **base Q** na etapa final do projeto.

