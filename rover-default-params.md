# Параметры для вшивания в прошивку Rover (сбор со скринов MP)

Прошивка: ArduRover V4.7.0-dev, ветка rover (05aca53e6c)

## Скрин 4 — Servo Output (skid steering)

| Параметр | Значение | Примечание |
|---|---|---|
| SERVO1_FUNCTION | 73 | ThrottleLeft |
| SERVO1_MIN | 988 | |
| SERVO1_TRIM | 1500 | |
| SERVO1_MAX | 2100 | |
| SERVO2_FUNCTION | 63 | RCIN13 (транзит с пульта) |
| SERVO2_MIN | 1100 | |
| SERVO2_TRIM | 1500 | |
| SERVO2_MAX | 1900 | |
| SERVO3_FUNCTION | 74 | ThrottleRight |
| SERVO3_MIN | 988 | |
| SERVO3_TRIM | 1500 | |
| SERVO3_MAX | 2100 | |
| SERVO4_FUNCTION | 62 | RCIN12 (транзит с пульта) |
| SERVO4_MIN | 1100 | |
| SERVO4_TRIM | 1500 | |
| SERVO4_MAX | 1900 | |
| SERVO5_FUNCTION … SERVO16_FUNCTION | 0 | Disabled, MIN/TRIM/MAX дефолт 1100/1500/1900 |
| SERVOx_REVERSED (все) | 0 | реверсы не включены |

## Скрин 3 — Serial Ports (Config > Serial Ports)

| Параметр | Значение | Примечание |
|---|---|---|
| SERIAL1_PROTOCOL | 2 | MAVLink2, UART7 (TELEM1, RTS/CTS Auto) |
| SERIAL1_BAUD | 115 | 115200 |
| SERIAL2_PROTOCOL | 2 | MAVLink2, UART1 |
| SERIAL2_BAUD | 115 | 115200 |
| SERIAL3_PROTOCOL | 5 | GPS, UART2 |
| SERIAL3_BAUD | 230 | 230400 |
| SERIAL4_PROTOCOL | 23 | RCIN, UART3 |
| SERIAL4_BAUD | 115 | 115200 |
| SERIAL5_PROTOCOL | -1 | None, UART8 |
| SERIAL5_BAUD | 57 | 57600 |
| SERIAL6_PROTOCOL | -1 | None, UART4 |
| SERIAL6_BAUD | 57 | 57600 |
| SERIAL7_PROTOCOL | -1 | None, UART6 |
| SERIAL7_BAUD | 57 | 57600 |
| SERIAL8_PROTOCOL | 2 | MAVLink2, OTG2 (USB, baud не важен) |

## Скрин 2 — Battery Monitor 1 (Setup > Optional Hardware > Battery Monitor)

| Параметр | Значение | Примечание |
|---|---|---|
| BATT_MONITOR | 0 | Disabled — первый монитор выключен |

## Скрин 1 — Battery Monitor 2 (Setup > Optional Hardware > Battery Monitor 2)

| Параметр | Значение | Примечание |
|---|---|---|
| BATT2_MONITOR | 4 | Analog Voltage and Current |
| BATT2_CAPACITY | 60000 | mAh |
| BATT2_VOLT_MULT | 21 | Voltage divider (Calced) |
| BATT2_VOLT_PIN | ? | на скрине не видно |
| BATT2_CURR_PIN | ? | на скрине не видно |
| BATT2_AMP_PERVLT | ? | на скрине не видно |
