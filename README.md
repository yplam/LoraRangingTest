# LoraRangingTest
Simple testing code for Lora ranging. SX1280, mbed and STM32 Nucleo-64 boards ( NUCLEO_F411RE as master and NUCLEO_L476RG as slave ).

![LoraRangingTest](https://raw.githubusercontent.com/yplam/LoraRangingTest/master/files/LoraRangingTest.jpg "LoraRangingTest")

## Hardware

![Schematic](https://raw.githubusercontent.com/yplam/LoraRangingTest/master/files/Schematic.png "Schematic")


## Getting started

```
git clone https://github.com/yplam/LoraRangingTest.git
mbed deploy
mbed compile -m NUCLEO_F411RE -t gcc_arm
mbed compile -m NUCLEO_L476RG -t gcc_arm
```

Master image: ./BUILD/NUCLEO_F411RE/GCC_ARM/LoraRangingTest.hex

Slave image: ./BUILD/NUCLEO_L476RG/GCC_ARM/LoraRangingTest.hex

## Pin config

mbed_app.json:

```
{
  "config": {
    "trace-level": {
      "help": "Options are TRACE_LEVEL_ERROR,TRACE_LEVEL_WARN,TRACE_LEVEL_INFO,TRACE_LEVEL_DEBUG",
      "macro_name": "MBED_TRACE_MAX_LEVEL",
      "value": "TRACE_LEVEL_INFO"
    }
  },
  "target_overrides": {
    "*": {
      "target.features_add": ["COMMON_PAL"],
      "mbed-trace.enable": true,
      "platform.stdio-convert-newlines": true
    },
    "NUCLEO_F411RE": {
      "platform.default-serial-baud-rate": 115200,
      "target.OUTPUT_EXT": "hex",
      "target.macros": [
        "LORA_MOSI=PA_7",
        "LORA_MISO=PA_6",
        "LORA_SCLK=PA_5",
        "LORA_NSS=PA_4",
        "LORA_BUSY=PC_8",
        "LORA_DIO1=PC_6",
        "LORA_RST=PA_12",

        "APP_HAS_LCD=1",
        "LCD_MOSI=PC_3",
        "LCD_MISO=PC_2",
        "LCD_SCLK=PB_10",
        "LCD_NSS=PB_12",
        "LCD_RESET=PC_10",
        "LCD_DC=PC_11",

        "APP_LED1=PC_9",
        "APP_LED2=PC_5",
        "APP_ENTITY_MODE=MASTER"
      ]
    },
    "NUCLEO_L476RG": {
      "platform.default-serial-baud-rate": 115200,
      "target.OUTPUT_EXT": "hex",
      "target.macros": [
        "LORA_MOSI=PA_7",
        "LORA_MISO=PA_6",
        "LORA_SCLK=PA_5",
        "LORA_NSS=PA_4",
        "LORA_BUSY=PC_8",
        "LORA_DIO1=PC_6",
        "LORA_RST=PA_12",
        "APP_LED1=PB_13",
        "APP_LED2=PC_5",
        "APP_ENTITY_MODE=SLAVE"
      ]
    }
  }
}

```


