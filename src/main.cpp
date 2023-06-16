/****************************************************************************************************************************
  UDPSendReceive.ino - Simple Arduino web server sample for SAMD21 running WiFiNINA shield
  For any WiFi shields, such as WiFiNINA W101, W102, W13x, or custom, such as ESP8266/ESP32-AT, Ethernet, etc

  WiFiWebServer is a library for the ESP32-based WiFi shields to run WebServer
  Based on and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Based on  and modified from Arduino WiFiNINA library https://www.arduino.cc/en/Reference/WiFiNINA
  Built by Khoi Hoang https://github.com/khoih-prog/WiFiWebServer
  Licensed under MIT license
 ***************************************************************************************************************************************/


#include "DebugPlatform.h"
#include "radio.h"
#include "Arduino.h"

#define SERIAL Serial

#define DEBUG_WIFI_WEBSERVER_PORT   Serial
#define SAMPLES 1024;
#define SENSOR_PERIOD 100

//magnetic
double threshold_m;
const double tolerance=0.15;
const int hall_sensor=A2;

enum magnet_state{
  MAG_NONE = 0,
  MAG_DOWN = 1,
  MAG_UP = 2, 
};

//extern std::queue<send_queue_t> send_queue;
extern QueueHandle_t  send_queue;

extern xSemaphoreHandle radioSemaphore;


unsigned long last;
unsigned long diff;
void isr(){
  unsigned long now = micros();
  diff = now - last;
  last = now;
}

TaskHandle_t conn_han, sensor_han, task_man_han;

//Server_Connection_Manager* conn_mann;
int interruptPin = 3;

void sensor_task();

void setupQueues(){
  send_queue =  xQueueCreate(20, sizeof(send_queue_t));

  radioSemaphore = xSemaphoreCreateBinary();
}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string buffer for task stats

void taskMonitor(void *pvParameters)
{
    int measurement;
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    while(1)
    {
    	SERIAL.flush();
		  SERIAL.println("");			 
    	SERIAL.println("****************************************************");
    	SERIAL.print("Free Heap: ");
    	SERIAL.print(xPortGetFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.print("Min Heap: ");
    	SERIAL.print(xPortGetMinimumEverFreeHeapSize());
    	SERIAL.println(" bytes");
    	SERIAL.flush();

    	SERIAL.println("****************************************************");
    	SERIAL.println("Task            ABS             %Util");
    	SERIAL.println("****************************************************");

    	vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    	SERIAL.println(ptrTaskList); //prints out already formatted stats
    	SERIAL.flush();

		SERIAL.println("****************************************************");
		SERIAL.println("Task            State   Prio    Stack   Num     Core" );
		SERIAL.println("****************************************************");

		vTaskList(ptrTaskList); //save stats to char array
		SERIAL.println(ptrTaskList); //prints out already formatted stats
		SERIAL.flush();

		SERIAL.println("****************************************************");
		SERIAL.println("[Stacks Free Bytes Remaining] ");

		measurement = uxTaskGetStackHighWaterMark( conn_han );
		SERIAL.print("Thread A: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( sensor_han );
		SERIAL.print("Thread B: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( task_man_han );
		SERIAL.print("Monitor Stack: ");
		SERIAL.println(measurement);

		SERIAL.println("****************************************************");
		SERIAL.flush();
    vTaskDelay( (1000 * 1000) / portTICK_PERIOD_US ); // print every 10 seconds
    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    vTaskDelete( NULL );

}


void setup()
{
  Serial.begin(115200);
  Serial1.begin(600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, RISING);
  while (!Serial && millis() < 5000);
  setupQueues();
  vSetErrorLed(LED_BUILTIN, HIGH);


  xTaskCreate((TaskFunction_t) conn_mann_task, "WiFi", 1024, NULL, tskIDLE_PRIORITY  + 2, &conn_han);
  xTaskCreate((TaskFunction_t) sensor_task, "Sensors", 1024, NULL, tskIDLE_PRIORITY  + 2, &sensor_han);
  //xTaskCreate((TaskFunction_t) radioDetect, "Radio", 512, NULL, tskIDLE_PRIORITY  + 2, NULL);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 2, &task_man_han);
  vSetErrorSerial(&Serial);
  Serial.println("Tasks Started");
    // Start the RTOS, this function will never return and will schedule the tasks.
  threshold_m = analogRead(hall_sensor) * (3.3/1023.0);

  Serial.print("threshold: ");
  Serial.println(threshold_m);
  vTaskStartScheduler();
  
  //conn_mann = new Server_Connection_Manager; //Start WiFi

}

unsigned long last_age_ping = 0;

char name_buf[10];
char last_valid_name[4];
int name_index = 0;
bool streaming_name;

void sensor_handler(){
  
  unsigned long now = millis();
  
  if ((now - last_age_ping) > SENSOR_PERIOD){
    last_age_ping = now;

    uint8_t* arr = new uint8_t[sizeof(unsigned long) + 4];
    send_queue_t metadata;
    metadata.buffer = arr;
    metadata.type = AGE;
    metadata.size = sizeof(unsigned long);
    arr[0] = AGE;
    arr[1] = 0;
    arr[2] = sizeof(unsigned long);
    arr[3] = 0;

    arr[4] = diff & 0xFF; 
    arr[5] = (diff >> 8) & 0xFF; 
    arr[6] = (diff >> 16) & 0xFF;
    arr[7] = (diff >> 24) & 0xFF;

    xQueueSendToBack(send_queue, &metadata, 10);

    //if (send_queue.size() < 20){
    //  send_queue.push(metadata);
    //}



    //magnetometer
    int sensor_in = analogRead(hall_sensor);

    //Serial.println(sensor_in);

    //ADC has a 10bit depth
    double voltage = sensor_in * (3.3/1023.0);

    //Serial.println(voltage);
    magnet_state mag_state;
    //inverting amplifier - only down when output is small
    if(voltage > threshold_m + tolerance){
        mag_state = MAG_DOWN;
    }
    else if(voltage < threshold_m - tolerance){
        mag_state = MAG_UP;
    }
    else{
        mag_state = MAG_NONE;
    }
    uint8_t* magarr = new uint8_t[1 + 4];
    send_queue_t magnetadata;
    magnetadata.buffer = magarr;
    magnetadata.type = MAGNET;
    magnetadata.size = 1;
    magarr[0] = MAGNET;
    magarr[1] = 0; //size H
    magarr[2] = 1; //Size L
    magarr[3] = 0;
    magarr[4] = (uint8_t) mag_state;
    xQueueSendToBack(send_queue, &magnetadata, 10);

    uint8_t* radioarr = new uint8_t[4+3];
    send_queue_t radio_data;
    radio_data.buffer = radioarr;
    radio_data.type = NAME;
    radio_data.size = 3;
    radioarr[0] = NAME;
    radioarr[1] = 0;
    radioarr[2] = 3;
    radioarr[3] = 0;
    //memcpy(&radioarr[4], &last_valid_name, 4);
    xQueueSendToBack(send_queue, &radio_data, 10);
    
  }
  /*
  if (Serial1.available()){
    char c = Serial1.read();
    Serial.print("Char Recv: ");
    Serial.println(c);
    if (c == "#"[0]){
      Serial.println("Delimiter");
      int len = name_index;
      name_index = 0;
      
    }
    name_buf[name_index] = c;
    name_index++;
    if (name_index >= 4){
      memcpy(&last_valid_name, name_buf, name_index); //Dest ptr, Source ptr, len in bytes
      Serial.println("Sent NAme");
    } 
    
  }
  */
}

void sensor_task(){
  while(1){
      sensor_handler();
  }
  vTaskDelete(NULL);
}





void loop(){

  //conn_mann->periodicHandler();
  //sensor_handler();
    
}
  

