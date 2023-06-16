#include <string>
#include <vector>
#include "Arduino.h"

#include "FreeRTOS_SAMD21.h"
#include "queue.h"
#include "DebugPlatform.h"
#include "radio.h"


xSemaphoreHandle radioSemaphore;
unsigned long last_sent;

#define RADIO_PERIOD 100

uint8_t radio_buf[4+6];
send_queue_t radio_pack;

// radio
void printString(std::string str) {
    for (int i = 0; i < str.size(); i++) {
        Serial.printf("%c", str[i]);
    }
    Serial.printf("\n");
}

int detectEdge(int prevBit, int bit) {
    if (prevBit == 0 && bit == 1) {
     return 1;
    } else if (prevBit == 1 && bit == 0) return -1;
    else return 0;
}

std::string decodeName(std::vector<int> &nameBits) {
    unsigned long now = millis();
    if ((now - last_sent) < RADIO_PERIOD){
        return "";
    }
    last_sent = now;
    std::string name = "";


    
    radio_pack.buffer = &radio_buf[0];
    radio_pack.size = 6;
    radio_pack.type = NAME;
    radio_buf[0] = NAME;
    radio_buf[1] = 0;
    radio_buf[2] = 0x06;
    radio_buf[3] = 0;
    int index = 4;
    for (int i = 0; i < 40; i += 10) {
        int val = 0;
        for (int j = 1; j < 9; j++) {
            val += pow(2, j - 1) * nameBits[i + j];
        }
        char c = static_cast<char>(val);
        name.push_back(c);
        radio_buf[index] = c;
        index++;
    }
    //xSemaphoreGive(radioSemaphore);
    
    return name;
}

std::string detectedName;
void radioDetect(){
    int bitCount = 0;
    int prevBit = 0;
    double low_threshold = 1.0;
    unsigned long lastEdgeTime = micros();
    std::vector<int> nameBits;
    bool started = false;
    bool ended = false;

    int maxCount = 0;

    while (true) {
        maxCount++;
        if (maxCount > 1000000) return;
        int VA0 = analogRead(A0);
        double level = double(VA0 * 3.3) / 1023.0;
        int curBit = level < low_threshold ? 0 : 1;

        int edge = detectEdge(prevBit, curBit);
        unsigned long curEdgeTime = micros();
        if (bitCount >= 40) {
            Serial.printf("Name: ");
            detectedName = decodeName(nameBits);
            printString(detectedName);

            ended = true;
            return;
        };

        if (edge != 0) {

            double difference = (curEdgeTime - lastEdgeTime) / 1000.0;

            int bit = edge == -1 ? 1 : 0;
            int numCycles = bit ? floor(difference / 1.66667) : ceil(difference / 1.66667);

            if (started) {
                for (int i = 0; i < numCycles; i++) {
                nameBits.push_back(bit);
                }
                 bitCount += numCycles;
            }

            if (!started && edge == -1 && numCycles > 10) {
                started = true;
                bitCount = 0;
            }

            lastEdgeTime = curEdgeTime;
        }

        prevBit = curBit;
        vTaskDelay(10 / portTICK_PERIOD_MS); //10ms Wait
    }
    vTaskDelete(NULL);
}
