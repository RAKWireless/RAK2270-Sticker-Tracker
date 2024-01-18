# RAK2270-Sticker-Tracker

Open source files for RAK2270 Sticker Tracker

## Product Description

RAK2270 is a sleek sticker tracker for LoRaWAN® designed like a label, making it easy to attach to any product. It offers real-time location tracking through the Helium network server's advanced features. With a built-in temperature sensor and an integrated 3-axis accelerometer, it provides accurate temperature monitoring and optimized device usage. It supports multiple LoRaWAN® bands, ensuring versatility for various regions and applications. The RAK2270 is a compact and powerful solution for precise and efficient tracking needs.

[RAK2270 works with Helium network and Trackpac platform](https://store.rakwireless.com/products/rak2270-rak-sticker-tracker) which allows you to quickly use the device with out doing any on-boarding process. Just scan the QR code and wait for the device to show on the Trackpac platform.

## Design Files

In this repository, we provide hardware and software files so you can have 100% access on the internals of RAK2270 Tracker Sticker. 

### Hardware

The RAK2270 is desiged using Cadence Allegro EDA software. We provided Cadence compatible files as well as the generated gerber file. The full schematic is in PDF file as well.

### Software

The main chip used on the RAK2270 is the WisDuo Module RAK3172-SiP/RAK3172LP-SiP (pin-to-pin and software compatible). This chip has STM32WL SoC which is a combination of MCU and LoRa/SubGhz transceiver. The firmware written on this is based on [RUI3](https://docs.rakwireless.com/RUI3/). You can start creating your own firmware using this [RAK3172-SiP guide](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-SiP/Quickstart/#rak3172-sip-as-a-stand-alone-device-using-rui3).

## Direct order on JLCPCB

RAKwireless and JLCPCB collaborate together so that you can order directly a RAK2270 with complete components from JLCPCB. You just have to use [this link](https://cart.jlcpcb.com/quote/?from=rak&fileUrl=https%3A%2F%2Fs3.eu-central-1.amazonaws.com%2Frakwireless-downloads-center-prod%2FLoRa%2FRAK2270%2Fjlcpcb%2FGERBER_FPC_RAK2270.zip&bomUrl=https%3A%2F%2Fs3.eu-central-1.amazonaws.com%2Frakwireless-downloads-center-prod%2FLoRa%2FRAK2270%2Fjlcpcb%2FBOM_PCB_RAK2270.xlsx&cplUrl=https%3A%2F%2Fs3.eu-central-1.amazonaws.com%2Frakwireless-downloads-center-prod%2FLoRa%2FRAK2270%2Fjlcpcb%2FPLACEMENT_FPC_RAK2270.xlsx) and just proceed each step of the order. The files will be automatically uploaded as you go on each step. This makes ordering of RAK2270 directly from JLCPCB easier and faster.