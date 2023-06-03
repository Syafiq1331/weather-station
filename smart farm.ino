#include <ESP8266WiFi.h> //Library untuk Terhubung ke Wi-Fi
#include "Adafruit_MQTT.h" //Library Adafruit MQTT
#include "Adafruit_MQTT_Client.h" //Library MQTT Client
#include <DHT.h> //Library DHT11
#include <MQUnifiedsensor.h> // Library untuk MQ0135
#define WLAN_SSID "KILLUA" //Nama Wi-Fi
#define WLAN_PASS "nada124@" //Password
#define AIO_SERVER "io.adafruit.com" //Broker atau Server
#define AIO_SERVERPORT 1883 //Port MQTT

#define AIO_USERNAME "nrkholid" //Username Adafruit IO
#define AIO_KEY "aio_SnkQ84U7YVWJDM8TaghpbWYJ2dZE" //Key Adafruit IO
#define Relay1 D0 //Pin Relay 1 di NodeMCU
#define Relay2 D1 //Pin Relay 2 di NodeMCU
#define LED D2 //Pin LED di NodeMCU
#define DHTPIN D4 //Pin DHT di NodeMCU
#define DHTTYPE DHT11 //Jenis DHT yang digunakan
#define placa "ESP8266" //Define jenis board yang digunakan
#define Voltage_Resolution 5 //Tegangan yang digunakan
#define pin A0 //Pin yang digunakan untuk MQ-135
#define type "MQ-135" //Jenis MQ yang digunakan
#define ADC_Bit_Resolution 10 //Resolusi Bit ADC
#define RatioMQ135CleanAir 3.6 //Nilai Udara dianggap bersih

DHT dht(DHTPIN, DHTTYPE); //Aktifkan fungsi DHT
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type); //Aktifkan fungsi MQ
char str_hum[8]; //Membuat Variabel Char str_hum dengan panjang 8 karakter
char str_temp[8]; //Membuat Variabel Char str_temp dengan panjang 8 karakter
char str_co[8]; //Membuat Variabel Char str_co dengan panjang 8 karakter
char str_co2[8]; //Membuat Variabel Char str_co2 dengan panjang 8 karakter
unsigned long durasiKirim = 0; //Untuk membantu fungsi Millis
unsigned long jedaKirim = 6000; //Untuk membantu fungsi Millis
WiFiClient client; //Mengaktifkan fungsi WiFi Client
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY); //Aktifkan fungsi MQTT dan check informasi ke Adafruit IO
Adafruit_MQTT_Subscribe lampu1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led"); //Fungsi untuk Subscribe ke Topik LED
Adafruit_MQTT_Subscribe lampu2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay1"); //Fungsi untuk Subscribe ke Topik Relay1

Adafruit_MQTT_Subscribe lampu3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay2"); //Fungsi untuk Subscribe ke Topik Relay2
Adafruit_MQTT_Publish suhu = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/suhu"); //Fungsi untuk Publish ke Topik Suhu
Adafruit_MQTT_Publish kelembaban = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/kelembaban"); //Fungsi untuk Publish ke Topik Kelembaman
Adafruit_MQTT_Publish KCO = Adafruit_MQTT_Publish(&mqtt,
AIO_USERNAME "/feeds/CO"); //Fungsi untuk Publish ke Topik CO 
Adafruit_MQTT_Publish KCO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CO2"); //Fungsi untuk Publish ke Topik CO2
void MQTT_connect(); //Untuk terhubung ke Adafruit MQTT

void setup() {
  Serial.begin(115200); //Baudrate untuk Komunikasi Serial
  pinMode(LED,OUTPUT); //Set LED sebagai Output
  pinMode(Relay1,OUTPUT); //Set Relay1 sebagai Output
  pinMode(Relay2,OUTPUT); //Set Relay2 sebagai Output
  digitalWrite(LED, LOW); //Set Nilai LED
  digitalWrite(Relay1, HIGH); //Set Nilai Relay1
  digitalWrite(Relay2, HIGH); //Set Nilai Relay2
  dht.begin(); //Memulai fungsi DHT
  MQ135.setRegressionMethod(1); //_PPM = a*ratio^b //Set Method
  yang digunakan untuk MQ-135
  MQ135.init(); //Memulai fungsi MQ-135
  //Fungsi untuk Kalibrasi MQ-135
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  delay(10);
  //Untuk memulai terhubung dengan jaringan Wi-Fi
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //Subscribe Topik Lampu1, Lampu2, dan Lampu3
mqtt.subscribe(&lampu1);
mqtt.subscribe(&lampu2);
mqtt.subscribe(&lampu3);
}

void loop() {
  unsigned long time = millis(); //Fungsi Millis
  MQTT_connect(); //Untuk fungsi reconnecr ke Adafruit MQTT
  MQ135.update(); //Update nilai sensor MQ-135
  //Pembacaan Nilai CO oleh Sensor MQ-135
  MQ135.setA(605.18); MQ135.setB(-3.937);
  float CO = MQ135.readSensor();
  //Pembacaan Nilai CO2 oleh Sensor MQ-135
  MQ135.setA(110.47); MQ135.setB(-2.862);
  float CO2 = MQ135.readSensor();
  //Pembacaan nilai Suhu dan Humidity oleh Sensor DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  //Konversi Variabel Float / Double ke String dengan 4 nilai di
  depan koma dan 2 nilai di belakang koma
  dtostrf(t, 4, 2, str_temp);
  dtostrf(h, 4, 2, str_hum);
  dtostrf(CO, 4, 2, str_co);
  dtostrf(CO2, 4, 2, str_co2);
  //Pengiriman Nilai Hasil Pembacaan Sensor ke Adafruit IO setiap 6 Detik

  if ((unsigned long)(time - durasiKirim) >= jedaKirim)
  {
    suhu.publish(str_temp);
    kelembaban.publish(str_hum);
    KCO.publish(str_co);
    KCO2.publish(str_co2);
    durasiKirim = millis();
  }
  //Menjalan fungsi subscribe
  Adafruit_MQTT_Subscribe *subscription;
  //Pembacaan Nilai berdasarkan Subscribe yang diterima
  while ((subscription = mqtt.readSubscription(5000))) {
  //Kondisi untuk menghidupkan LED apabila subscribe yang diterima adalah LED
    if (subscription == &lampu1) {
      Serial.print(F("Got: "));
      Serial.println((char *)lampu1.lastread);
      if(strcmp((char*)lampu1.lastread,"ON")==0){
        digitalWrite(LED,HIGH);
      }
      if(strcmp((char*)lampu1.lastread,"OFF")==0){
        digitalWrite(LED,LOW);
      }
    }
    //Kondisi untuk menghidupkan Relay1 apabila subscribe yang diterima adalah Relay1
    if (subscription == &lampu2) {
      Serial.print(F("Got: "));
      Serial.println((char *)lampu2.lastread);
      if(strcmp((char*)lampu2.lastread,"ON")==0){
        digitalWrite(Relay1,LOW);
      }
      if(strcmp((char*)lampu2.lastread,"OFF")==0){
        digitalWrite(Relay1,HIGH);
      }
    }

    //Kondisi untuk menghidupkan Relay2 apabila subscribe yang diterima adalah Relay2
    if (subscription == &lampu3) {
      Serial.print(F("Got: "));
      Serial.println((char *)lampu3.lastread);
      if(strcmp((char*)lampu3.lastread,"ON")==0){
        digitalWrite(Relay2,LOW);
      }
      if(strcmp((char*)lampu3.lastread,"OFF")==0){
        digitalWrite(Relay2,HIGH);  
      }
    }
  }
}

//Fungsi untuk koneksi ke Adafruit MQTT
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retry = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reconnecting . . .");
    mqtt.disconnect();
    delay(5000);
    retry--;
    if (retries == 0) {
      while (1);
    } 
  }
  Serial.println("MQTT Connected!");
}
