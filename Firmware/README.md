# RAK2270-Sticker-Tracker

## Software

The main chip used on the RAK2270 is the WisDuo Module RAK3172-SiP/RAK3172LP-SiP (pin-to-pin and software compatible). This chip has STM32WL SoC which is a combination of MCU and LoRa/SubGhz transceiver. The firmware written on this is based on [RUI3](https://docs.rakwireless.com/RUI3/). You can start creating your own firmware using this [RAK3172-SiP guide](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-SiP/Quickstart/#rak3172-sip-as-a-stand-alone-device-using-rui3).

### Install

[Install RUI3 Board Support Package in Arduino IDE](https://docs.rakwireless.com/RUI3/Supported-IDE/#software-setup)  
Board: RAKwireless RUI STM32 Boards, WisDuo RAK3272-SiP Board, version 4.1.0 or later  

### Upgrade firmware

- Use "Upload" by Arduino IDE to upgrade firmware  
- Use J-Link to upgrade firmware  
  Firmware image will be generated in C:\Users\$USER\AppData\Local\Temp\arduino_build_XXXXX  
  "$USER" is your user name on Windows  
  "arduino_build_XXXXX" will be generate when Verify by Arduino IDE  
  Programming "RAK2270.ino.hex" by J-Link

### Setting LoRaWAN parameters

Use [RUI3 AT Command](https://docs.rakwireless.com/RUI3/Serial-Operating-Modes/AT-Command-Manual/)

### Factory default value

- LoRaWAN Region: US915, AT+BAND=5  
- Channel mask: use channel 8-15, AT+MASK=0002  
- ADR disable, AT+ADR=0  
- Unconfirmed package, AT+CFM=0  
- TX Power: 0, AT+TXP=0  
- Sending interval: 60 mintues, ATC+SENDFREQ=60  
- DEVEUI, APPEUI, APPKEY will be generated during factory production  

### Uplink payload format

The data packet is encoded by CayenneLPP format based on the [_**CayenneLPP format**_](https://github.com/ElectronicCats/CayenneLPP) provided by ElectronicCats.

The channel ID's used for the different values are:  

| Channel Name     | Channel ID | Type        | Value                              |
| ---------------- | ---------- | ----------- | ---------------------------------- |
| LPP_TEMPERATURE  | 0          | Temperature | RAK2270 Temperature in 1/10 °C     |
| LPP_ANALOG_INPUT | 0          | Analog      | RAK2270 Bettery voltage in 1/100 V |

The packet sent out displayed as hex values looks like this:  
0x00 0x67 0x01 0x11 0x00 0x02 0x01 0x2a  

| Bytes | Meaning                  | Value in Hex | Value   |
| ----- | ------------------------ | ------------ | ------- |
| 1     | RAK2270 Channel ID       | 0x00         |         |
| 2     | Temperature Channel Type | 0x67         |         |
| 3, 4  | Temperature Value        | 0x01 0x11    | 27.3 °C |
| 5     | RAK2270 Channel ID       | 0x00         |         |
| 6     | Analog Channel Type      | 0x02         |         |
| 7, 8  | Battery Voltage Value    | 0x01 0x2a    | 2.98 V  |

### Downlink command format

- Modify Periodic Uplink Interval Command  

| Field | Size | Description                    |
| ----- | ---- | ------------------------------ |
| Type  | 1    | 0x02                           |
| Data  | 2    | Range: 0x0001 - 0x05A0, mintue |

- Modify Confirmed Command

| Field | Size | Description                        |
| ----- | ---- | ---------------------------------- |
| Type  | 1    | 0x06                               |
| Data  | 1    | 0x00: Uncomfirmed, 0x01: Comfirmed |


