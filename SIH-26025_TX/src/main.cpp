#include <Arduino.h>
#include "IntegratedAlgorithm.h"
#include "LoRa_Packetizer.h"
#include "LoRaManager.h"
#include "Isolation_Forest_Wrapper.h"

// Hardware Pin Definitions
#define IMU_SDA_PIN     21
#define IMU_SCL_PIN     22
#define VIBE_PIN        25
#define GPS_RX_PIN      16
#define GPS_TX_PIN      17

#define LORA_SS_PIN     5
#define LORA_RST_PIN    27
#define LORA_DIO0_PIN   26
#define LORA_FREQ_HZ    433E6

// Module Instantiations
IntegratedAlgorithm algo(IMU_SDA_PIN, IMU_SCL_PIN, VIBE_PIN, GPS_RX_PIN, GPS_TX_PIN);
LoRaPacketizer packetizer;
LoRaManager lora(LORA_FREQ_HZ, LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

// --- Global Shared Variables (Updated in Background) ---
portMUX_TYPE anomalyMutex = portMUX_INITIALIZER_UNLOCKED;

volatile int   g_lastVerdict = 1;      // Default: 1 (NORMAL)
volatile float g_lastScore   = 0.0f;   // Continuous background score

// --- FreeRTOS Background Task (Runs continuously on Core 0) ---
void backgroundAnomalyWorker(void *pvParameters) {
    LoRaPacketizer tempPacketizer;

    for (;;) {
        // 1. Pack current sensor data snapshot from algo
        tempPacketizer.packData(algo);
        SensorPacket currentPkt = tempPacketizer.getPacket();

        // 2. Compute Isolation Forest score and verdict
        int newVerdict = iforestPredict(currentPkt);
        float newScore = iforestScore(currentPkt);

        // 3. Update global shared variables safely
        portENTER_CRITICAL(&anomalyMutex);
        g_lastVerdict = newVerdict;
        g_lastScore   = newScore;
        portEXIT_CRITICAL(&anomalyMutex);

        // 4. Yield execution brief period to feed Core 0 Watchdog Timer
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Timer for 1 Hz LoRa transmission
uint32_t lastTxTime = 0;
const uint32_t TX_INTERVAL_MS = 1000;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("--- Initializing System with Background Anomaly Worker ---"));

    // 1. Initialize IMU, Vibration, and GPS
    algo.begin();

    // 2. Initialize LoRa Module
    if (!lora.begin(/*txPower=*/17, /*spreadingFactor=*/7, /*bandwidth=*/125E3)) {
        Serial.println(F("Critical Error: LoRa hardware setup failed! Halting..."));
        while (1);
    }

    // 3. Perform initial score computation before worker starts
    packetizer.packData(algo);
    SensorPacket initialPkt = packetizer.getPacket();
    g_lastVerdict = iforestPredict(initialPkt);
    g_lastScore   = iforestScore(initialPkt);

    // 4. Launch background worker on Core 0 (Main loop runs on Core 1)
    xTaskCreatePinnedToCore(
        backgroundAnomalyWorker,  // Worker function
        "AnomalyWorker",           // Task name
        16384,                     // Stack size in bytes
        NULL,                      // Task parameters
        1,                         // Priority
        NULL,                      // Task handle
        0                          // Pinned to Core 0
    );

    Serial.println(F("Setup complete. Streaming data & updating score continuously in background..."));
}

void loop() {
    // Continuously update hardware sensors (IMU, Vibration, GPS UART)
    algo.update();

    // Rate-limited 1 Hz LoRa Transmission
    if (millis() - lastTxTime >= TX_INTERVAL_MS) {
        lastTxTime = millis();

        // 1. Safely fetch the last updated background variables
        int currentVerdict;
        float currentScore;

        portENTER_CRITICAL(&anomalyMutex);
        currentVerdict = g_lastVerdict;
        currentScore   = g_lastScore;
        portEXIT_CRITICAL(&anomalyMutex);

        // 2. Pack current sensor data and set the last updated anomaly values
        packetizer.packData(algo);
        packetizer.setAnomalyResults(currentVerdict, currentScore);

        Serial.printf("[TX Stream] Last Score: %.4f | Verdict: %s\n", 
                      currentScore, (currentVerdict == 1) ? "NORMAL" : "ANOMALY");

        // 3. Transmit packet over LoRa
        bool txStatus = lora.sendPacket(packetizer);

        if (txStatus) {
            Serial.print(F("[LoRa TX] Packet Sequence: "));
            Serial.println(packetizer.getPacket().packetSequence);
        } else {
            Serial.println(F("[LoRa TX Error]"));
        }
    }

    delay(1);
}