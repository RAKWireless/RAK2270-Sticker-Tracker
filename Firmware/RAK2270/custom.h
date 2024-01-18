#ifndef __CUSTOM_H__
#define __CUSTOM_H__


//Version string
#define FW_VERSION    "1.1.0"
#define CUSTOM_MODEL  "RAK2270"
#define CUSTOMER      "Standard"
#define CUSTOME_VER   CUSTOM_MODEL"_"FW_VERSION"_"CUSTOMER

//the channel for Cayenne LPP
#define TEMP_CH     0
#define BAT_CH      0

//LoRaWAN configure
#define LORAWAN_CONFIRMED         false
#define LORAWAN_RETRY             0
//FPort
#define FPORT_PERIODIC_UPLINK     10

//Interval
#define INTERVAL_PERIODIC_UPLINK  60

#endif
