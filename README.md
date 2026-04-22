ESP32 with cam module showcase

Ensure PlatformIO CLI is installed. If not, install it via pip: 

`pip install platformio`

Open a terminal and navigate to the project directory: 

`cd espcam`

Run the build command: 

`pio run`

The project built without errors, using 26.7% of flash and 15.3% of RAM. The output files are in the esp32s3 directory. If you need to upload to the ESP32, use 

`pio run --target upload`

Change WiFi credentials if you want to serve on exising network:
main.cpp:

`const char *WIFI_SSID = "PUT YOUR WIFI SSID HERE";`

`const char *WIFI_PASS = "PUT YOUR WIFI PASSWORD HERE";`

Otherwise it will work as access point with this credentials:
main.cpp:

`'WiFi.softAP("ESP32CAM", "12345678");'`
