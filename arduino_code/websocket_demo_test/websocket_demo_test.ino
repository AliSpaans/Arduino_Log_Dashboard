#include <MsgPack.h>
#include <ArduinoHttpClient.h>
#include "arduino_secrets.h"
#include <WebSocketClient.h>
#include <b64.h>
#include <WiFiNINA_Generic.h>

#define MAX_BUFFER_SIZE 128
#define MAX_BYTE_BUFFER 93

<<<<<<< HEAD:arduino_code/websocket_demo_test/websocket_demo_test.ino
#define Current_sensor A7  //The current sensor analog input pin
#define Voltage_sensor1 A6  //The 1st voltage sensor analog input pin
#define Voltage_sensor2 A1 //The 2nd voltage sensor analog input pin
=======
#define PATH "/device/" DEVICE_KEY
>>>>>>> 5e74878022b28ecac6a6d8bf7dce9a15cb33577a:arduino_code/websocket_demo_test.ino

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
<<<<<<< HEAD:arduino_code/websocket_demo_test/websocket_demo_test.ino
char path[] = SECRET_PATH;
char host[] = SECRET_HOST;

float Cur;
float v[101];
float c[101];
=======
char path[] = PATH;
char host[] = "167.71.68.242";
>>>>>>> 5e74878022b28ecac6a6d8bf7dce9a15cb33577a:arduino_code/websocket_demo_test.ino

WiFiClient client;
WebSocketClient webSocketClient = WebSocketClient(client, host, 80);

int status = WL_IDLE_STATUS;

void setup() {
<<<<<<< HEAD:arduino_code/websocket_demo_test/websocket_demo_test.ino
  analogReference(AR_EXTERNAL);
  Serial.begin(9600);
  pinMode(Current_sensor, INPUT);
  pinMode(Voltage_sensor1, INPUT);
  pinMode(Voltage_sensor2, INPUT);
=======
    Serial.begin(9600);
>>>>>>> 5e74878022b28ecac6a6d8bf7dce9a15cb33577a:arduino_code/websocket_demo_test.ino
  
    while (!Serial);
    while ( status != WL_CONNECTED) {
        Serial.print("Attempting to connect to Network named: ");
        Serial.println(ssid);     // print the network name (SSID);
        status = WiFi.begin(ssid, pass);
        delay(5000);
    }

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
  
    webSocketClient.begin(path);

    int messagesize = webSocketClient.parseMessage();
    if(messagesize > 0) {
        Serial.println(webSocketClient.readString());
    }
}

bool Write_Message(WebSocketClient client,
                   float *voltage, float *current,
                   uint8_t samples) {

    MsgPack::arr_t<float> v;
    MsgPack::arr_t<float> c;

    for(int i=0; i<samples; i++) {
        v.push_back(voltage[i]);
        c.push_back(current[i]);
    }

    MsgPack::fix_arr_t<MsgPack::arr_t<float>, 2> m {v, c};
    MsgPack::Packer packer;

    packer.serialize(m);

    char message[MAX_BUFFER_SIZE];

    unsigned int length = packer.size();
    unsigned int index = 0;
    const uint8_t *data = packer.data();
    unsigned int encoded_length;

    while ( length > MAX_BYTE_BUFFER ) {

        encoded_length = b64_encode(data + index,
                                    MAX_BYTE_BUFFER,
                                    (unsigned char*)message,
                                    MAX_BUFFER_SIZE);

        message[encoded_length] = '\0';
        client.beginMessage(TYPE_TEXT);

        unsigned int send_length = encoded_length+1;

        do {
            int write_len = client.write((uint8_t *)message, encoded_length+1);
            send_length = send_length - write_len;
        } while(send_length > 0);

        client.endMessage();
        length = length - MAX_BYTE_BUFFER;
        index = index + MAX_BYTE_BUFFER;
    }

    if (length > 0) {
        encoded_length = b64_encode(data + index,
                                    length,
                                    (unsigned char *)message,
                                    MAX_BUFFER_SIZE);

        message[encoded_length] = '\0';
        client.beginMessage(TYPE_TEXT);
        unsigned int send_length = encoded_length+1;

        do {
            int write_len = client.write((uint8_t *)message,
                                         encoded_length+1);
            send_length = send_length - write_len;
        } while(send_length > 0);

        client.endMessage();
    }

    return true;
}

void LoadData(void){

 
    for(int i=0;i<101;i++){
      
      int voldata1 = analogRead(Voltage_sensor1);
      int voldata2 = analogRead(Voltage_sensor2);

      int vol_1_center = voldata1-1023;
      int vol_2_center = 1023-voldata2;

      float volR01 = (vol_1_center / 1024.0) * 3.3 * 156.545;
      float volR02 = (vol_2_center / 1024.0) * 3.3 * 156.545;
  
      int CurRO = analogRead(Current_sensor)-527;
      Cur =  (( ((float)CurRO) * 3.3)/1024.0)*10.22;
            
            c[i]=Cur;
      Serial.print(Cur);
      Serial.print("\t");
      if(voldata1<voldata2){
         v[i]=volR01;
          Serial.println(volR01);
          } else {
            v[i]=volR02;
            Serial.println(volR02);
          }  
         

      delay(9);
      
     }
}



void loop() {
    LoadData();
    //Serial.println("finished loading data!");
    Write_Message(webSocketClient, v, c, 101);

}
