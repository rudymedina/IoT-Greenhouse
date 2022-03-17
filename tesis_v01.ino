#include <SoftwareSerial.h>;
#include "DHT.h"
#include <OneWire.h>                
#include <DallasTemperature.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//declaras variables
float t1,t2;
float h1,h2; 
float valorLDR = 0;

int pinLDR = 6;
int pinLDR2 = 7;
int pinLDR3 = 8;

int humidity3 = analogRead(A0);
int humidity4 = analogRead(A1);
int humidity5 = analogRead(A2);


#define pin1 26
#define pin2 27

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 10000;
unsigned long interval2 = 10000;
unsigned long interval3 = 10000;
unsigned long interval4 = 10000;
unsigned long interval5 = 10000;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis(); 
unsigned long prevTime_T4 = millis(); 
unsigned long prevTime_T5 = millis();

//PINES PARA CONEXION ESP32-----------
OneWire ourWire(4);                
DHT dht1(pin1, DHT11);    
DHT dht2(pin2, DHT11);
DHT dht3(pin3, DHT11);
SoftwareSerial myCo2(23, 5); // RX, TX
byte request[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];
//----------------------------------
// Configuraciones del sistema----------------------
              //Contraseña de vuestro router.
#define WLAN_SSID       "Honor 5C" 
#define WLAN_PASS       "314159265" 
#define MQTT_SERVER      "35.194.15.126" 
#define MQTT_SERVERPORT  1883
#define MQTT_USERNAME    ""
#define MQTT_KEY         ""
#define MQTT_PUBLIC_TEMP_0   "Experiment/Romensa/Temperature_1" 
#define MQTT_PUBLIC_HUMI_0   "Experiment/Romensa/Humidity_1" 
#define MQTT_PUBLIC_TEMP_1   "Experiment/Romensa/Temperature_2"
#define MQTT_PUBLIC_HUMI_1   "Experiment/Romensa/Humidity_2"
#define MQTT_PUBLIC_TEMP_2   "Experiment/Romensa/Temperature_3"
#define MQTT_PUBLIC_HUMI_2   "Experiment/Romensa/Humidity_3"
#define MQTT_PUBLIC_CO2      "Experiment/Romensa/co2"

#define MQTT_PUBLIC_LUZ_0   "Experiment/Romensa/luz_0"
#define MQTT_PUBLIC_LUZ_1   "Experiment/Romensa/luz_1"
#define MQTT_PUBLIC_LUZ_2   "Experiment/Romensa/luz_2"

#define MQTT_PUBLIC_HUMI_3   "Experiment/Romensa/Humidity_4" 
#define MQTT_PUBLIC_HUMI_4   "Experiment/Romensa/Humidity_5" 
#define MQTT_PUBLIC_HUMI_5   "Experiment/Romensa/Humidity_6"

#define MQTT_PUBLIC_SERVO_0   "Experiment/Romensa/servo_0"
#define MQTT_PUBLIC_SERVO_1   "Experiment/Romensa/servo_1"



//----------------------------------
// conexion y rutamiento de topics para los sensores--------
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_USERNAME, MQTT_KEY);
Adafruit_MQTT_Publish temp_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_0);
Adafruit_MQTT_Publish hum_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_0);
Adafruit_MQTT_Publish temp_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_1);
Adafruit_MQTT_Publish hum_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_1);
Adafruit_MQTT_Publish temp_2_hdt_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_2);
Adafruit_MQTT_Publish hum_2_hdt_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_2);
Adafruit_MQTT_Publish co2_z19 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_CO2);

Adafruit_MQTT_Publish luz_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_LUZ_0);
Adafruit_MQTT_Publish luz_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_LUZ_1);
Adafruit_MQTT_Publish luz_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_LUZ_2);

Adafruit_MQTT_Publish hum_2_hdt_3 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_3);
Adafruit_MQTT_Publish hum_2_hdt_4 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_4);
Adafruit_MQTT_Publish hum_2_hdt_5 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_5);
//-----------------------------------------------------------------------------------------------
//arrancar sensores y conexiones-----------------
void setup() {
  Serial.begin(115200);
  Serial.println("*******Dary Malinovky*******");
  connectWiFi();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  connectMQTT();
  dht1.begin();
  dht2.begin();
  ds18.begin();
  myCo2.begin(9600);
}
//------------------------------------------------- 
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime_T1 > interval) {
    leerdht1();
    prevTime_T1 = currentTime;
    }
  if (currentTime - prevTime_T2 > interval) {
    leerdht2();
    prevTime_T2 = currentTime;
    }
  if (currentTime - prevTime_T3 > interval) {
    leerds18();
    prevTime_T3 = currentTime;
    }
  if (currentTime - prevTime_T4 > interval) {
    leerco2();
    prevTime_T4 = currentTime;
    }
  if (currentTime - prevTime_T5 > interval) {
    Serial.println("отправленные данные MQTT успешно");
    Serial.println("______________________________________");
    prevTime_T5 = currentTime;
    }
  checkWifi();
  reconnect();
  
}
void leerdht1() {  
    t1 = dht1.readTemperature();
    h1 = dht1.readHumidity();
  
  Serial.print("Temperatura DHT_01: ");
  Serial.print(t1);
  Serial.println(" ºC.");
  Serial.print("Humedad DHT_01: ");
  Serial.print(h1);
  Serial.println(" %."); 
  temp_0_hdt_0.publish(t1);
  hum_0_hdt_0.publish(h1);
}
void leerdht2() { 
  t2 = dht2.readTemperature();
  h2 = dht2.readHumidity();
  
  Serial.print("Temperatura DHT_02: ");
  Serial.print(t2);
  Serial.println(" ºC.");
  Serial.print("Humedad DHT_02: ");
  Serial.print(h2);
  Serial.println(" %.");
  temp_1_hdt_1.publish(t2);
  hum_1_hdt_1.publish(h2); 
}
void leerdht3() { 
  t3 = dht3.readTemperature();
  h3 = dht3.readHumidity();
  
  Serial.print("Temperatura DHT_03: ");
  Serial.print(t3);
  Serial.println(" ºC.");
  Serial.print("Humedad DHT_03: ");
  Serial.print(h3);
  Serial.println(" %.");
  temp_2_hdt_2.publish(t3);
  hum_2_hdt_2.publish(h3); 
}

void leerluz_1() { 
  valorLDR = analogRead(pinLDR);
  Serial.print("luz_01 = ");
  Serial.println(valorLDR);
  luz_0.publish(valorLDR);
}
void leerluz_2() { 
  valorLDR2 = analogRead(pinLDR2);
  Serial.print("luz_01 = ");
  Serial.println(valorLDR2);
  luz_1.publish(valorLDR2);
}
void leerluz_3() { 
  
  valorLDR3 = analogRead(pinLDR3);
  Serial.print("luz_01 = ");
  Serial.println(valorLDR3);
  luz_2.publish(valorLDR3);
}

void humidity3() {

   int humedad3 = analogRead(humidity3);
   Serial.print(humedad3);
  
   if(humedad < 500)
   {
      Serial.println("Encendido");  
      //hacer las acciones necesarias
   }
   delay(1000);
}
void humidity4() {

   int humedad4 = analogRead(humidity4);
   Serial.print(humedad4);
  
   if(humedad < 500)
   {
      Serial.println("Encendido");  
      //hacer las acciones necesarias
   }
}
void humidity5() {

   int humedad5 = analogRead(humidity5);
   Serial.print(humedad5);
  
   if(humedad < 500)
   {
      Serial.println("Encendido");  
      //hacer las acciones necesarias
   }
}

void bonba1() {
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);   // encendemos el rele osea la bomba
  delay(1000);                       // esperamos 1 segundo
  digitalWrite(LED_BUILTIN, LOW);    // apagamos la bomba 
  delay(1000);                       // esperamos un segundo
}
void bonba2() {
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);   // encendemos el rele osea la bomba
  delay(1000);                       // esperamos 1 segundo
  digitalWrite(LED_BUILTIN, LOW);    // apagamos la bomba 
  delay(1000);                       // esperamos un segundo
}
void bonba3() {
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);   // encendemos el rele osea la bomba
  delay(1000);                       // esperamos 1 segundo
  digitalWrite(LED_BUILTIN, LOW);    // apagamos la bomba 
  delay(1000);                       // esperamos un segundo
}

void leerco2() { 
  myCo2.write(request, 9);
  memset(response, 0, 9);
  myCo2.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error");
  } else {
    unsigned int HLconcentration = (unsigned int) response[2];
    unsigned int LLconcentration = (unsigned int) response[3];
    unsigned int co2 = (256*HLconcentration) + LLconcentration;
    Serial.println(co2);
    for (i = 0; i < 9; i++) {
      Serial.print("0x");
      Serial.print(response[i],HEX);
      Serial.print("  ");
    }
    Serial.println("  ");
    co2_z19.publish(co2); 
  }  
}
void connectWiFi() {
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
     attempts++;
     if (attempts > 30) {
      ESP.restart();
      }     
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void checkWifi() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("(void checkWifi) Reconnecting to WiFi...");
    WiFi.disconnect();
    //WiFi.reconnect();
    ESP.restart();
    previousMillis = currentMillis;
  }
}
//------------------------------------------------------------------
void connectMQTT() {
  if (mqtt.connected())
    return;
  Serial.print("Connecting to MQTT... ");
  while (mqtt.connect() != 0) {
       Serial.println("Error. Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);
  }
  Serial.println("MQTT Connected!");

}
void reconnect() {
// test is mqtt connected
  if (!client.connected()) {
    Serial.println("(void reconnect) mqtt desconect reset modul");
    ESP.restart();
  }
}

 
