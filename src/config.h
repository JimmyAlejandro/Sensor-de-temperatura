/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "JimmyQuiroz"
#define IO_KEY       "aio_hhor66dEglYQfgeFicndHH7qyUwh"

/******************************* WIFI **************************************/


#define WIFI_SSID "iPhone de Jimmy"
#define WIFI_PASS "jimmy862"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
