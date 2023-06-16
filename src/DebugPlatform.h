#ifndef DEBUGPLATFORM_H_
#define DEBUGPLATFORM_H_

#include "inttypes.h"
#include <queue>
#include "FreeRTOS_SAMD21.h"
#include "queue.h"

enum packet_type {
    DRIVE = 1,
    HANDSHAKE = 2,
    AGE = 3,
    MAGNET = 4,
    NAME = 5,
};

struct send_queue_t{
    uint8_t* buffer;
    packet_type type;
    int size;
};



struct drive_packet{
    int FL;
    int FR;
    int BL;
    int BR;

};

void conn_mann_task();

class Server_Connection_Manager{
public:
    Server_Connection_Manager();
    void periodicHandler();
    int status;     // the Wifi radio's status
    bool client_known;
    
    
private: 
    uint16_t remotePort;
    char packetBuffer[255];          // buffer to hold incoming packet

};






#endif