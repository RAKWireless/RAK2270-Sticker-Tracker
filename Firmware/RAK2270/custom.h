#ifndef __CUSTOM_H__
#define __CUSTOM_H__


//Version string
#define FW_VERSION    "1.1.2"
#define CUSTOM_MODEL  "RAK2270"
#define CUSTOMER      "Generic"
#define CUSTOME_VER   CUSTOM_MODEL"_"FW_VERSION"_"CUSTOMER

//the channel for Cayenne LPP
#define TEMP_CH     0
#define BAT_CH      0

//LoRaWAN configure
#define LORAWAN_RETRY             0
//FPort
#define FPORT_PERIODIC_UPLINK     10
#define FPORT_DOWNLINK_ACK        11
#define FPORT_MD_EVENT_UPLINK     20

//Interval
#define INTERVAL_PERIODIC_UPLINK  60
#define INTERVAL_JOIN             240
#define INTERVAL_LINKCHECK        60

#endif
