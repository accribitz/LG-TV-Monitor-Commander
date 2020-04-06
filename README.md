# LG TV/Monitor Commander

Control LG TV/Monitor using Arduino's serial port or SoftSerial.

## Features

* Control LG TV/Monitor using Arduino.
* Save control commands to EEPROM.
* Change saved control command without compile and flash.
* Send control command using buttons or serial port communication.

## Requirements

### Arduino

* One open hardware serial port or SoftSerial support
* Flash 16KB or Higher
* SRAM 1KB or Higher
* EEPROM PIN * 8 bytes or Higher (If you use save function.)

### Serial Cable and Connection

~~~
GND connections are omitted.

                 5V         12V      LG TV/Monitor Phone Plug
+---------+   +-------------------+     |---------------\
|LG_TX_PIN|-->|TTL-IN   RS-232-OUT|------------->   |    \
| Arduino |   |  Level Converter  |     | GND | TXD | RXD >
|LG_RX_PIN|<--|TTL-OUT   RS-232-IN|<-------------------  /
+---------+   +-------------------+     |---------------/
~~~

Default Pin assign

|Arduino|LG_RX_PIN|LG_TX_PIN|CONTROL_RX_PIN|CONTROL_TX_PIN|
|:--:|:--:|:--:|:--:|:--:|
|UNO|10|11|0 (Shared Serial Port)|1 (Shared Serial Port)|
|Pro Micro|1|0|USB Serial Port|USB Serial Port|

### Buttons ( If you needed. )

~~~
                +----+
                |    |
+---------+   +-+----+-+
|    PIN_X|-->|  Push  |
| Arduino |   |        |
|      GND|<--| Button |
+---------+   +--------+
~~~

## Install

1. If your Arduino does NOT have SERIAL_PORT_MONITOR, change CONTROL_RX_PIN and CONTROL_TX_PIN if you needed.  
    SERIAL_PORT_MONITOR is serial port for communicate to PC.  
    Most of Arduino do not need change, because of they have SERIAL_PORT_MONITOR.  
    See your Arduino's "pins_arduino.h".  
    If your Arduino does not have SERIAL_PORT_MONITOR, default CONTROL_RX_PIN is 14 and CONTROL_TX_PIN is 15.
1. If your Arduino does NOT have SERIAL_PORT_HARDWARE_OPEN, change LG_RX_PIN and LG_TX_PIN if you needed.  
    SERIAL_PORT_HARDWARE_OPEN is free serial port for connect other serial devices.  
    Maybe, most of Arduino do not need change. Default LG_RX_PIN is 10 and LG_TX_PIN is 11.  
    See SoftSerial [limitations](https://www.arduino.cc/en/Reference/SoftwareSerial).
1. Compile sketch and flash to Arduino.

## How to use

1. Create serial cable. See [Serial Cable and Connection](#Serial-Cable-and-Connection).  
1. ( If you need ) Connect but	tons to Arudino empty pins and GND.  
1. Connect Arduino to LG TV/Monitor using Serial Cable and level converter.  
1. Connect Arduino to your computer and use Arduino IDE Serial Monitor.  
1. Input [commands](#Commands), and test this program.  
    1. Save your reuired commands.  
        **! BE CAREFUL !**  
        **DO NOT use any pins assigned to serial ports ( SERIAL_PORT_MONITOR and SERIAL_PORT_HARDWARE_OPEN ).**  
        UNO : SERIAL_PORT_MONITOR use PIN 0 and 1. No SERIAL_PORT_HARDWARE_OPEN, using SoftSerial, LG_RX_PIN is 10 and LG_TX_PIN is 11.  
        Pro Micro : SERIAL_PORT_HARDWARE_OPEN, PIN 0 and PIN 1.  
    1. Send your saved commands.  
1. ( If you connected buttons ) Push connected buttons and test it.

## Commands

~~~
Commands : Description
    help : Show this help.
    spec : Show board specifications.
    show : Show saved commands.
    save : Save command.
           save[ ][PIN_NUMBER][ ][Command1][Command2][ ][Set ID][ ][Data]
           Example 1 : Save power off command to PIN 2 EEPROM. "save 2 ka 00 00"
           Example 2 : Clear PIN 2 EEPROM data. "save 2"
    send : Send saved command.
           send[ ][PIN_NUMBER]
           Example   : Send PIN 2 command. "send 2"
    pass : Pass through input.
           pass[ ][Command1][Command2][ ][Set ID][ ][Data]
           Example   : Send power off command. "pass ka 00 00"
   clear : Clear all saved commands.
  export : Show saved control commands for copy & paste.
~~~


## Tested devices

- Arduino
    - ATMega328P
        - [ELEGOO UNO R3](https://www.elegoo.com/product/elegoo-uno-r3-board-atmega328p-atmega16u2-with-usb-cable/) ( Arduino UNO compatible )
    - ATMega32U4
        - SparkFun Pro Micro compatible
    - ATmega168V
        - [Gakken Japanino](https://otonanokagaku.net/japanino/) ( Arduino Duemilanove compatible without clock )

- TV/Monitor
   - TV
        - Nothing ( I have NO LG TVs. )
   - Monitor
        - 43UD79-B

If you tested another devices, please report it.


## References

- https://minkara.carview.co.jp/userid/2475954/blog/40076687/
- https://www.avforums.com/threads/lg-tv-with-rs232c-mini-jack.1978458/
- https://forum.mysensors.org/topic/2965/lg-tv-controller/
- LG's TV/Monitor Manuals

## License
Copyright (c) 2020 accribitz  
This software is released under the GPLv3 License, see LICENSE.  
This website content is released under the CC BY 4.0 License, see LICENSE-CC.
