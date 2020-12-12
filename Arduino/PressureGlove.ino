#include <Wire.h>

//For Wi-Fi ESP8266
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
  #define BroadPin(x) D##x
  
#elif defined(ARDUINO_linkit_7697)  //For Linkit 7697
  #include <LWiFi.h>
  #include <WiFiUdp.h>
  #define BroadPin(x) x
  
#else
  #error "Not Support board"
#endif


const char * WiFi_SSID = "YOUR_SSID";  //Connect to this SSID
const char * WiFi_Password = "YOUR_PASSWORD";  //Connect with Password
const char * DeviceName = "RightHandPressure";  //DeviceName, send to PC. Must Match DeviceName in PC.
const int dstPort = 4444; //Udp Port
//const char * mDNSName = "RightHandPressure";
const int ClientMaxTimeout = 30;  //Timeout.

WiFiClient Socket_Client;
WiFiUDP udp;
//

//For 16-bit Resolution ADC
#include "ADS1115.h"
#ifdef ESP8266
const int8_t _pin_SDA = BroadPin(4);  //Pin D4 in ESP8266
const int8_t _pin_SCL = BroadPin(5);  //Pin D5 in ESP8266
#endif
ADS1115 adc;  //Other boards don't have Wire.begin(SDA, SCL)
//

//For 16-Channel Mux
#include "CD74HC4067.h"
const int8_t _pin_s0 = BroadPin(0);
const int8_t _pin_s1 = BroadPin(1);
const int8_t _pin_s2 = BroadPin(2);
const int8_t _pin_s3 = BroadPin(3);
CD74HC4067 Mux(_pin_s0, _pin_s1, _pin_s2, _pin_s3);
int8_t Max_Sensor_Count = 16;  //Max Pin Count
byte SendBuffer[8 * 16];
//

//Get Position in Buffer
inline byte* GetSendBufferPtr(int8_t SensorID)  
{
  return &SendBuffer[8 * SensorID];
}

//Init Buffer. Init Const in Packet. Reduce Writing
void InitSendBuffer() 
{
  //Packet Def:
  //FF FE 08 MUX ADC aH aL 00
  for (int8_t i = 0; i < Max_Sensor_Count; ++i)
  { 
    byte *Buffer = GetSendBufferPtr(i);
    Buffer[0] = 0xFF;
    Buffer[1] = 0xFE;
    Buffer[2] = 0x08;
    Buffer[3] = (byte)i;
    Buffer[7] = 0x00;
  }
}

//Write Data to Buffer
void SetSendBuffer(int8_t SensorID, int8_t ADC_MUX, int16_t adcValue) 
{
  byte *Buffer = GetSendBufferPtr(SensorID);
  Buffer[4] = ADC_MUX;
  Buffer[5] = (byte)(adcValue >> 8);
  Buffer[6] = (byte)(adcValue & 0xFF);
}

//Return bool: Is Receive UDP Broadcast
bool UdpRecv()
{
  static bool Result = false;
  if (Result) //If has been received
  {
    Result = false; //Reset
    udp.begin(dstPort); //Restart UDP Broadcast Listener
    return false;
  }
  int pktSize = udp.parsePacket();  //Check is Packet Available
  if (pktSize >= 4) //Packet is 4 Byte Long
  {
    byte pktBuf[pktSize];
    IPAddress target(udp.remoteIP()); //Get Broadcast Source IP (ServerIP)
    udp.read(pktBuf, pktSize);  //Read Packet
    int Port = pktBuf[2] | pktBuf[3] << 8;  //Read Port from Packet
    Serial.print("Recv: ");
    Serial.print(target);
    Serial.print(" Port ");
    Serial.println(Port);
    if (pktBuf[0] == 0xFF && pktBuf[1] == 0xEE) //Check Packet Header
    {
      Socket_Client.connect(target, Port);  //Create Socket Connection
      Result = true;  //Received Broadcast!
    }
    udp.stop(); //Stop Listener
    if (!Result)  //If Error occurs
      udp.begin(dstPort); //Start Listener
    Serial.println(Result ? "Success" : "Failed");
  }
  return Result;
}

void setup() 
{
#ifdef ESP8266
  //WiFi Setting
  wifi_set_sleep_type(NONE_SLEEP_T); //Disable Sleep
  wifi_set_opmode(STATION_MODE);  //Set as Station Only Mode (Without SoftAP)
  WiFi.setPhyMode(WIFI_PHY_MODE_11N); //Using 11N
  //
#endif

  //Serial Setting
  Serial.begin(115200);
  Serial.println("Please Wait");
  //
  
#ifdef ESP8266
   Max_Sensor_Count = 4;  //Set Real Pin Count
#elif defined(ARDUINO_linkit_7697)
  pinMode(BroadPin(10), INPUT); //Read Real Pin Count from Pin
  pinMode(BroadPin(11), INPUT);
  pinMode(BroadPin(12), INPUT);
  pinMode(BroadPin(13), INPUT);
  int SensorCountFromPin = 0; //4-Bit -> 0~15.
  if (digitalRead(BroadPin(10)) == HIGH)
    SensorCountFromPin |= 1;
  if (digitalRead(BroadPin(11)) == HIGH)
    SensorCountFromPin |= 2;
  if (digitalRead(BroadPin(12)) == HIGH)
    SensorCountFromPin |= 4;
  if (digitalRead(BroadPin(13)) == HIGH)
    SensorCountFromPin |= 8; 
  Max_Sensor_Count = ++SensorCountFromPin;  //(0 + 1) ~ (15 + 1) -> 1~16
#endif

  Serial.print("Max Sensor Supported: ");
  Serial.println(Max_Sensor_Count);
  
  //Setting ADC Module
#ifdef ESP8266
  adc.begin(_pin_SDA, _pin_SCL);
#elif defined(ARDUINO_linkit_7697)
  adc.begin();  //SDA P9 SCL P8
#endif

  adc.reset();
  delay(1000);
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS); //Set Data Per Second
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT); //Read Voltage when Call trigger_sample()
  adc.set_pga(ADS1115_PGA_ONE);  //+-4.096V. Warning: Do not apply more than VDD + 0.3 V
  adc.set_mux(ADS1115_MUX_GND_AIN0);
  InitSendBuffer();
  //
  
  //Trying to connect
  WiFi.begin(WiFi_SSID, WiFi_Password);
  while (WiFi.status() != WL_CONNECTED)
    delay(100);
  Serial.println("Connected");
  Serial.println(WiFi.localIP());

//  //Set mDNS || Mac/iOS: Support || Windows: Not Support ||
//  if (MDNS.begin(mDNSName)) //Set mDNS. Ex: localIP <--> esp8266.local
//    Serial.println("Success mDNS setting!");
//  else
//    Serial.println("Fail mDNS setting!");
//  //

  udp.begin(dstPort); //Start UDP Broadcast Listener
} 

void loop() 
{
  static int8_t Mux_Index = 0;
  static int8_t ADC_Mux = 0;
  if (!Socket_Client.connected()) //If client is not connected
  {
    if (!UdpRecv()) //If No UDP Broadcast
      return;
    Socket_Client.write(DeviceName);  //Tell Server(PC) who are you
    Socket_Client.flush();
    
#ifdef ESP8266
    Socket_Client.setNoDelay(true); //Prevent Data Latency
#else
    delay(1000);
#endif

    Socket_Client.setTimeout(ClientMaxTimeout);
  }

  //FF FE 08 MUX ADC aH aL 00
  if (Socket_Client && Socket_Client.connected()) //If Socket_Client exist && Socket_Client is connected
  {
    //Read Data & Send
    if (adc.trigger_sample() != 0)
    {
      Serial.println("ERROR");
      return;
    }
    yield();  //Give Control Back to OS (Socket, Internel Handler, etc)
    while (adc.is_sample_in_progress()) //Wait ADC Result
      ;
    int16_t adcValue = adc.read_sample(); //Read ADC Result
    SetSendBuffer(Mux_Index, ADC_Mux, adcValue);  //Write Result to Buffer
    
    //Switch to Next Sensor
    if (++Mux_Index == Max_Sensor_Count)  //If Restart from 0
    {
      Mux_Index = 0;
      Socket_Client.write(SendBuffer, 8 * Max_Sensor_Count);  //Send Packet from Buffer
    }
      
    Mux.channel(Mux_Index); //Change MUX Channel
    delayMicroseconds(1);    
  }
}
