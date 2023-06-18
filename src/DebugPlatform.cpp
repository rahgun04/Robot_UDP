
#include "DebugPlatform.h"


#define USE_WIFI_NINA         false
#define USE_WIFI101           true

// If not USE_WIFI_NINA, you can USE_WIFI_CUSTOM, then include the custom WiFi library here
#define USE_WIFI_CUSTOM       false


#include "WiFiUdp.h"
#include <WiFiWebServer.h>
#include <WiFiMDNSResponder.h>
#include "radio.h"

QueueHandle_t  send_queue;
QueueHandle_t  radio_queue;

//std::queue<send_queue_t> send_queue;

extern xSemaphoreHandle radioSemaphore;
extern send_queue_t radio_pack;


// Motor
const int pinDIR_L = 11; // Connected to DIR on the motor driver left
const int pinEN_L = 12; // Connected to EN on the motor driver left
const int pinDIR_R = 13;
const int pinEN_R = 6;

unsigned int localPort = 1883;
char mdnsName[] = "wifi102";
IPAddress ip(192, 168, 0, 5);

const char ssid[] = "Pixel 6"; //"Dell G15";//"Jordon";        // your network SSID (name)
const char pass[] =  "nointernet"; //"x2dwlx2d";        // your network password

WiFiMDNSResponder mdnsResponder;
WiFiUDP Udp;
IPAddress remoteIp;
Server_Connection_Manager* conn_mann;

void conn_mann_task(){
    conn_mann = new Server_Connection_Manager; //Start WiFi
    pinMode(pinDIR_L, OUTPUT);
    pinMode(pinEN_L, OUTPUT);
    pinMode(pinDIR_R, OUTPUT);
    pinMode(pinEN_R, OUTPUT);
    while(1){     
        conn_mann->periodicHandler();
    }
    vTaskDelete(NULL);
}

void drive_handler(drive_packet dp){
    if (dp.FL > dp.BL){
        digitalWrite(pinDIR_L, HIGH);
        analogWrite(pinEN_L, dp.FL);
    }else{
        digitalWrite(pinDIR_L, LOW);
        analogWrite(pinEN_L, dp.BL);
    }

    if (dp.FR > dp.BR){
        digitalWrite(pinDIR_R, HIGH);
        analogWrite(pinEN_R, dp.FR);
    }else{
        digitalWrite(pinDIR_R, LOW);
        analogWrite(pinEN_R, dp.BR);
    }
}


void Server_Connection_Manager::periodicHandler(){
    mdnsResponder.poll();
    //Serial.println("Periodic");

    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();

    if (packetSize){
        
        // read the packet into packetBufffer
        int len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
            packetBuffer[len] = 0;
        }
        if (packetBuffer[0] == 0x01){ //Drive Packet
            //Serial.println("Recieved Drive packet");
            drive_packet dp;
            dp.FL = packetBuffer[4];
            dp.FR = packetBuffer[5];
            dp.BL = packetBuffer[6];
            dp.BR = packetBuffer[7];
            drive_handler(dp);
            //Serial.println(dp.FL);
            
        }else if ((packetBuffer[0] == 0x02) && (packetBuffer[4] == 0xff)){ 
            Serial.println("Handshake Made");       
            remoteIp = Udp.remoteIP();
            remotePort = Udp.remotePort();
            client_known = true;
        }
    }
    
        
    
    if (client_known){
        for (int i = 0; i < 1; i++){
            /*
            if (!send_queue.empty()){
                send_queue_t m = send_queue.front();
                send_queue.pop();
                Udp.beginPacket(remoteIp, remotePort);
                Udp.write(m.buffer, m.size + 4);
                Udp.endPacket();
                delete [] m.buffer;
            }
            */
           send_queue_t buf;
           if(xQueueReceive(send_queue, &buf, 0) == pdPASS){
                Udp.beginPacket(remoteIp, remotePort);
                Udp.write(buf.buffer, buf.size + 4);
                Udp.endPacket();
                delete [] buf.buffer;
           }
                   
        }
    }
    //vTaskDelay(10 / portTICK_PERIOD_MS);

}

Server_Connection_Manager::Server_Connection_Manager(){
    
    Serial.print(F("Connecting to SSID: "));
    Serial.println(ssid);
    WiFi.config(ip);
    status = WiFi.begin(ssid, pass);

    delay(1000);

    // attempt to connect to WiFi network
    while ( status != WL_CONNECTED)
    {
        delay(500);
        // Connect to WPA/WPA2 network
        status = WiFi.status();

    }

    Serial.println(F("\nStarting connection to server..."));
    // if you get a connection, report back via serial:
    Udp.begin(localPort);

    Serial.print(F("Listening on port "));
    Serial.println(localPort);

        // Setup the MDNS responder to listen to the configured name.
    // NOTE: You _must_ call this _after_ connecting to the WiFi network and
    // being assigned an IP address
    
    if (!mdnsResponder.begin(mdnsName)) {
        Serial.println("Failed to start MDNS responder!");
        while(1);
    }else{
        Serial.print("Server listening at http://");
        Serial.print(mdnsName);
        Serial.println(".local/");
    }
    


}


