# Slicetext Modbus RTU Master Example

This example inherits from https://github.com/espressif/esp-idf/blob/master/examples/protocols/modbus/serial/mb_master.

Shows how to read all 6 characteristics from TH-MB-XXX-DS sensors from [Slicetex](https://slicetex.com/).

My wiring:

                                                                      +5V
                                                                       ^
                        +-----------------------------------------+    |
            GPIO25 -----| RO                                  Vcc |----+----- Brown
                        |                                         |
            GPIO26 -----| DI               MAX485               A |---------- Yellow (or Green)
    ESP32               |               TTL to RS485              |
    BOARD   GPIO33 -+---| DE                                    B |---------- Blue              TH-MB-XXX-DS
                    |   |                                         |
                    +---| /RE                                 GND |----+----- Black
                        +-----------------------------------------+    |
                                                                       v
                                                                      GND

Outputs:

    I (555) modbus_example: Characteristic #0 Humidity: 31.9 %
    I (1595) modbus_example: Characteristic #1 Temperature: 27.1 C
    I (2635) modbus_example: Characteristic #2 Humidity calibration: 0.0 %
    I (3675) modbus_example: Characteristic #3 Temperature Calibration: 0.0 C
    I (4715) modbus_example: Characteristic #4 ModBus address: 1
    I (5755) modbus_example: Characteristic #5 ModBus baud rate: 4800 bauds
