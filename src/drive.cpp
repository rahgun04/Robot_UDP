/*
#include "DebugPlatform.h"

drive_packet dp;


void drive_task(){
    while (1){
        // See if there's a message in the queue (do not block)
        if (xQueueReceive(drive_packet_queue, (void *)&dp, 0) == pdTRUE) {
            Serial.print("Packet: ");
            Serial.print(dp.x);
            Serial.println(dp.y);
        }
    }
    vTaskDelete(NULL);
}
*/