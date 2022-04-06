#include <samd.h>
#include <MsgPack.h>
#include <ArduinoHttpClient.h>
#include "arduino_secrets.h"
#include <WebSocketClient.h>
#include <b64.h>
#include <SPI.h>
#include <WiFiNINA.h>

#define MAX_BUFFER_SIZE 4096
#define MAX_BYTE_BUFFER 3000

//PINOUT ADC READ OUT
#define CURRENTSENSE 0x00  //The current sensor analog input pin
#define VOLTAGESENSE1 0x12  //The 1st voltage sensor analog input pin
#define VOLTAGESENSE2 0x0A //The 2nd voltage sensor analog input pin

#define VOLTAGECORRECTION  (3.3 * 193.545)
#define VOFFSET 4056
#define FILTERORDER 2
#define BUFFERSIZE 100

//Secret keys in seperate header file
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char path[] = SECRET_PATH;
char host[] = SECRET_HOST;

//Initial setup data buffers
float v[2][100];
float c[2][100];
float p[2][100];

//Initial setup wifi/websocketclient
WiFiClient client;
WebSocketClient webSocketClient = WebSocketClient(client, host, 80);
int status = WL_IDLE_STATUS;

//Timer counter 4 handler starting the ADC
void TC4_Handler(void){
  ADC->SWTRIG.bit.START = 1;
  TC4->COUNT32.INTFLAG.bit.OVF = 1;
}

//Set up ADC Handler
volatile int DoubleBuffer_full = 0;
volatile int DoubleBuffer_loc = 0;

//ADC handler that will load all the data in different buffers
void ADC_Handler(void){
  static uint16_t DownSampleBuffer[3][10];                            //3 buffers of each 10 to store data while the ADC is filling them all up
  static uint8_t Index = 0;                                           //indicator if all buffers are filled
  static uint8_t BufferIndex = 0;                                     //Indicator for the switch between the different buffers
  static const int pos[3]={CURRENTSENSE,VOLTAGESENSE1,VOLTAGESENSE2}; //Rotating pinout buffer
  static uint16_t TempVoltage;                                        //Temporary uint16_t to compare to other voltage input
  static volatile int DoubleIndex=0;                                  //Indicator for which buffer to fill - One is used to write while the other is used to send
  static volatile int DoubleBufferIndex = 0;
  
  if(DoubleIndex >= BUFFERSIZE) {
    DoubleBuffer_full = 1;
    DoubleBuffer_loc = DoubleBufferIndex;
    DoubleBufferIndex = (DoubleBufferIndex + 1) % 2;
    DoubleIndex = 0;
  }

    DownSampleBuffer[BufferIndex][Index]=ADC->RESULT.reg;
    uint16_t SecVoltage;
 
    if(Index==9){
      switch(BufferIndex){
        case 0:
          c[DoubleBufferIndex][DoubleIndex] = (((DownSampleBuffer[BufferIndex][0])-2100)/4096.0)*3.3*11;
          
          break;
        case 1:
          TempVoltage = DownSampleBuffer[BufferIndex][0];
          break;
        case 2:
          SecVoltage = DownSampleBuffer[BufferIndex][0];
          //Compare is done to eventually plant one set of data instead of 2 different data sets
          if(TempVoltage < SecVoltage){
            v[DoubleBufferIndex][DoubleIndex] = ((TempVoltage - VOFFSET)/4096.0) * VOLTAGECORRECTION;
          }else{
            v[DoubleBufferIndex][DoubleIndex] = ((VOFFSET - SecVoltage)/4096.0) * VOLTAGECORRECTION; //Inverting this set of data for the positive sinus
          }
          //Powercalculation is done by a simple P=U*I
          p[DoubleBufferIndex][DoubleIndex] = (c[DoubleBufferIndex][DoubleIndex])*(v[DoubleBufferIndex][DoubleIndex]);
          DoubleIndex++;
          
          break;
      }
    }
    
    //Reset buffer rotation
    if(BufferIndex==2){
      BufferIndex=0;
      Index=(Index+1)%10;
    }else{
      BufferIndex++;
    }

    ADC->INPUTCTRL.bit.MUXPOS = pos[BufferIndex];        //Pin rotation after every cycle
}


void setup() {
  digitalWrite(Reset, HIGH);
  delay(200); 
  pinMode(Reset, OUTPUT);
   
  Serial.begin(9600);
  delay(4000);          //Delay set for serial communication to start
  
  //Pinmode differint I/O
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);

  //General clock set for use in the Timer Counter and ADC
  GCLK->CLKCTRL.reg = (uint16_t) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(0x1C));
  while(GCLK->STATUS.reg & ( 1 << 7 ));
  GCLK->CLKCTRL.reg = (uint16_t) ( GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(0X1E));
  while(GCLK->STATUS.reg & ( 1 << 7 ));
  Serial.println("enabled clock");
  
  //Timer counter software reset
  TC4->COUNT32.CTRLA.reg = 1;
  //Wait till counter is done
  while(TC4->COUNT32.STATUS.reg & ( 1 << 7 ));
  
  TC4->COUNT32.CTRLA.bit.PRESCALER = 0x0;     //48MHz Clock
  TC4->COUNT32.CTRLA.bit.MODE = 0x2;          //Counter in 32-bit mode
  TC4->COUNT32.CTRLBCLR.bit.ONESHOT = 0x1;    //Disable one-shot counter
  TC4->COUNT32.CC[0].reg = 15840;             //3 times 8080Hz for the triple ADC measurement
  TC4->COUNT32.CTRLA.bit.WAVEGEN = 0x1;       //Waveform generator will match frequency
  TC4->COUNT32.CTRLBCLR.bit.DIR = 1;          //Counter is counting down from given count to zero
  TC4->COUNT32.EVCTRL.bit.OVFEO = 0x1;        //Overflow enabled
  TC4->COUNT32.INTENSET.bit.OVF = 0x1;        //Interrupt will be set at overflow
  TC4->COUNT32.CTRLA.bit.ENABLE = 0x1;        //Enable the timer counter

  //Enable timer counter handler
  NVIC_EnableIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);
  
  //Wait till counter is done
  while(TC4->COUNT32.STATUS.reg & ( 1 << 7 )); 

  //ADC Software reset
  ADC->CTRLA.bit.SWRST = 1;
  //Wait till ADC is done
  while(ADC->CTRLA.bit.SWRST);
  
  ADC->AVGCTRL.bit.ADJRES = 0x3;      //Division coefficient set to 8 for average
  ADC->AVGCTRL.bit.SAMPLENUM = 0x3;   //Samples to be collected set to 8 for average
  ADC->REFCTRL.bit.REFSEL = 0x3;      //VREF Pin Set to external
  ADC->INPUTCTRL.bit.MUXNEG = 0x18;   //Negative Mux input set to GND
  ADC->EVCTRL.bit.RESRDYEO = 1;       //Result ready output enabled
  ADC->CTRLB.bit.PRESCALER = 0x4;     //DIV64 prescaler
  ADC->CTRLB.bit.RESSEL = 0x1;        //16 bit resolution used for averaging
  ADC->CTRLB.bit.FREERUN = 0;         //Freerun disabled
  ADC->SAMPCTRL.bit.SAMPLEN = 15;     //Sampling time set to 15

  ADC->INTENSET.bit.RESRDY = 1;       //Result ready interrupt enabled
  ADC->CTRLA.bit.ENABLE = 1;          //Enable the ADC
  ADC->INPUTCTRL.bit.MUXPOS = 0x00;   //Positive MUX input set to AIN0 to start

  //Enable ADC handler
  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 0);

  //enable handling of interrupts
  __enable_irq(); 

    //Wifi connection setup
    while ( status != WL_CONNECTED) {
        Serial.print("Attempting to connect to Network named: ");
        Serial.println(ssid);                                     // print the network name (SSID);
        status = WiFi.beginEnterprise(ssid, SECRET_USER, pass);   //Use credentials to connect to accesspoint
    }
    
    //Disable low power mode of the WiFiNina module
    WiFi.noLowPowerMode();

    long rssi = WiFi.RSSI();
    Serial.print("Signal strenght:  ");
    Serial.println(rssi);
    
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
  
    //Open websocket connection with client
    webSocketClient.begin(path);

    //Wait for OK feedback
    int messagesize = webSocketClient.parseMessage();
    if(messagesize > 0) {
        Serial.println(webSocketClient.readString());
    }
   
}

//Function that will write the different buffers to the client
bool Write_Message(WebSocketClient client,
                   float *voltage, float *current, float *power,
                   uint8_t samples) {
                    

    int error;
    
    //
    MsgPack::arr_t<float> v;
    MsgPack::arr_t<float> c;
    MsgPack::arr_t<float> p;

    //
    for(int i=0; i<samples; i++) {
        v.push_back(voltage[i]);
        c.push_back(current[i]);
        p.push_back(power[i]);
    }

    //
    MsgPack::fix_arr_t<MsgPack::arr_t<float>, 3> m {v, c, p};
    MsgPack::Packer packer;

    packer.serialize(m);

    char message[MAX_BUFFER_SIZE];

    unsigned int length = packer.size();
    unsigned int index = 0;
    const uint8_t *data = packer.data();
    unsigned int encoded_length;

    bool first=true;

    while ( length > MAX_BYTE_BUFFER ) {

        encoded_length = b64_encode(data + index,
                                    MAX_BYTE_BUFFER,
                                    (unsigned char*)message,
                                    MAX_BUFFER_SIZE);

        message[encoded_length] = '\0';

        client.beginMessage(TYPE_TEXT);

        int write_len = client.print(message);
        Serial.println(write_len);

        error=client.endMessage();
        length = length - MAX_BYTE_BUFFER;
        index = index + MAX_BYTE_BUFFER;
    }

    if (length > 0) {
        encoded_length = b64_encode(data + index,
                                    length,
                                    (unsigned char *)message,
                                    MAX_BUFFER_SIZE);

        message[encoded_length]='\0';
        Serial.println(client.beginMessage(TYPE_TEXT));
        int write_len = client.print(message);
        
        Serial.println(client.endMessage());  
    }

    packer.clear();
    
    int messagesize = webSocketClient.parseMessage();
    if(messagesize > 0) {
      if(strcmp(webSocketClient.readString().c_str(), "OK")) {
          client.flush();
          return false;
      }
    }

    client.flush();

    return true;
}



void loop() {
  //While loop to send over data to the client if this parameters are matched
  while(client.connected() || WiFi.RSSI()>-78) {//WiFinina module automaticly disconnects if signal strenght is <-78
    if(DoubleBuffer_full) {
        DoubleBuffer_full = 0;
        long rssi = WiFi.RSSI();
        Write_Message(webSocketClient, v[DoubleBuffer_loc], c[DoubleBuffer_loc], p[DoubleBuffer_loc], 100);
    }
  }

  //Reconnecting to client and accespoint
  webSocketClient.beginMessage(TYPE_CONNECTION_CLOSE);
  webSocketClient.endMessage();
  webSocketClient.stop();
  WiFi.disconnect();
  WiFi.end();
  int status = WL_IDLE_STATUS;
  while(status != WL_CONNECTED) {
    status = WiFi.beginEnterprise(ssid, SECRET_USER, pass);
    long rssi = WiFi.RSSI();
    delay(2000);
  }
  Serial.println(webSocketClient.begin(path));

  //Arduino reset
  while(WiFi.RSSI()==0){
    digitalWrite(Reset, LOW);
  }
}
