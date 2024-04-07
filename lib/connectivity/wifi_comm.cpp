#include "wifi_comm.h"

const char *ssid = "No internet";
const char *password = "@jamalpur@";

void connectToWiFi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Attempting to connect to Wi-Fi...");
    }

    Serial.println("Connected to Wi-Fi successfully!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}