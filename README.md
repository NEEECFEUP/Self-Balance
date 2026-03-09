# Self Balance

O projeto implementa um controlador PID com base na distância a uma bola e ajusta a posição do servo para manter a distância próxima do **setpoint** definido.

Um **joystick** permite alterar o setpoint e ajustar parâmetros do controlador (Ki e Kd) em tempo real. Um **LCD 16x2 RGB** mostra informação do sistema.

---

## Funcionalidades

- Controlo **PID em tempo real**
- Sensor de distância **VL53L0X**
- Ajuste de **setpoint com joystick**
- Ajuste em tempo real de **Ki e Kd**
- **Filtro passa-baixo** na leitura do sensor
- **Filtro na derivada** para reduzir ruído
- Visualização em **LCD 16x2**
- Debug via **Arduino Serial Plotter**
- Frequência de controlo: **100 Hz**

---

## Hardware

- Arduino
- Sensor **VL53L0X**
- Servo da Hiwonder
- LCD **RGB 16x2 (I2C)**
- Joystick analógico
- Botões de reset

---

## Bibliotecas

Instalar no Arduino IDE:

- `SoftwareSerial`
- `Wire`
- `DFRobot_VL53L0X`
- `ServoHiwonder`
- `rgb_lcd`

---

## Ligações

| Componente | Pino |
|-------------|------|
| Servo Controller RX | 2 |
| Servo Controller TX | 3 |
| Joystick (setpoint) | A1 |
| Botão joystick | 5 |
| Ajuste Ki | A0 |
| Reset Ki | 4 |
| Ajuste Kd | A2 |
| Reset Kd | 6 |
| VL53L0X | I2C |
| LCD RGB | I2C |

---

## Parâmetros PID

```cpp
float kp = 0.55;
float ki = 0.32;
float kd = 0.4;
