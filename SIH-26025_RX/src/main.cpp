#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include "LoRa_Decoder.h"
#include "Supabase_Client.h"

// --- Wi-Fi & Supabase Configuration ---
const char* WIFI_SSID       = "Airtel_ravi";
const char* WIFI_PASSWORD   = "Ishu@#25";

const char* SUPABASE_URL    = "https://umhkwsyytcweuoftyhuc.supabase.co";
const char* SUPABASE_KEY    = "sb_publishable_6QR2I77tMKJK4Wxv-mpCyQ_0WIJVt3X";

// --- Hardware Pin Configurations ---
#define LORA_SS_PIN     5
#define LORA_RST_PIN    27
#define LORA_DIO0_PIN   26
#define LORA_FREQ_HZ    433E6 

#define RX_GPS_RX_PIN   16
#define RX_GPS_TX_PIN   17
#define RX_GPS_BAUD     9600

// --- Class Instances ---
HardwareSerial gpsSerial(2);
TinyGPSPlus rxGps;
LoRaDecoder decoder;
SupabaseClient supabase(WIFI_SSID, WIFI_PASSWORD, SUPABASE_URL, SUPABASE_KEY);

void processLocalGPS() {
    while (gpsSerial.available() > 0) {
        rxGps.encode(gpsSerial.read());
    }
}

void checkIncomingLoRa() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    // Check packet size against decoder expected structure size
    if (packetSize == decoder.getExpectedSize()) {
        uint8_t rxBuffer[sizeof(SensorPacket)];
        int bytesRead = 0;

        while (LoRa.available() && bytesRead < packetSize) {
            rxBuffer[bytesRead++] = (uint8_t)LoRa.read();
        }

        if (decoder.parseBuffer(rxBuffer, bytesRead)) {
            int rssi = LoRa.packetRssi();
            float snr = LoRa.packetSnr();

            // 1. Print decoded telemetry to local Serial
            decoder.printTelemetry(rssi, snr);

            // 2. Extract decoded struct
            DecodedTelemetry telemetry = decoder.getTelemetry();

            // 3. Send payload to Supabase database
            double rxLat = rxGps.location.isValid() ? rxGps.location.lat() : 0.0;
            double rxLon = rxGps.location.isValid() ? rxGps.location.lng() : 0.0;

            supabase.sendTelemetryToSupabase(telemetry, rxLat, rxLon);
        } else {
            Serial.println(F("[LoRa RX Error] Buffer parsing failed!"));
        }
    } else {
        // Size mismatch warning
        Serial.print(F("[LoRa RX Warning] Expected "));
        Serial.print(decoder.getExpectedSize());
        Serial.print(F(" bytes, but got "));
        Serial.print(packetSize);
        Serial.println(F(" bytes! Packet dropped."));

        while (LoRa.available()) LoRa.read();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("--- Initializing LoRa RX + GPS + Supabase Node ---"));

    // 1. Initialize Wi-Fi & Supabase connection
    supabase.beginWiFi();

    // 2. Initialize Local GPS
    gpsSerial.begin(RX_GPS_BAUD, SERIAL_8N1, RX_GPS_RX_PIN, RX_GPS_TX_PIN);

    // 3. Initialize LoRa Hardware
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQ_HZ)) {
        Serial.println(F("Critical Error: LoRa setup failed!"));
        while (1);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);

    Serial.print(F("[LoRa] Listening on 433 MHz. Expected packet size: "));
    Serial.print(decoder.getExpectedSize());
    Serial.println(F(" bytes."));
}

void loop() {
    processLocalGPS();
    checkIncomingLoRa();
    delay(1);
}