//Weather Station Sketch - Abdurrahman Aliyu Gambo Feb. 2024

#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Anemometer
#define AnemPin 39
#define TotalAnemRead 2000

//Gas Sensors
#define MQ7Pin 34
#define MQ2Pin 35
#define MQ135Pin 36
float MQ2R0 = 6983.38 ;
float MQ7R0 = 2991.92;
float MQ135R0 = 2115.56;

//ThingsBoard Connection
#define MaxConnectAttempts 20
const char* ssid = "*****";
const char* ssid2 = "*****";
const char* password = "*****";
const char* password2 = "*****";
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

//DHT11
static const int DHT_SENSOR_PIN = 17;
DHT_Unified dht(DHT_SENSOR_PIN, DHT11);

//BMP
Adafruit_BMP085 bmp;

//Value Holders
float gas, air, carbonmonoxide, temperature, pressure, humidity, windspeed, altitude = 0;
String jsonoutput;
bool wifi_connect(const char* SSID, const char* PASSWORD){
  
  
  int DCCounter = 0;
  WiFi.begin(SSID, PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    DCCounter++;
    if(DCCounter == MaxConnectAttempts){
      WiFi.disconnect();
      return false;
    }
  }
  return true;
}
bool Connect(bool NetChoice){
  Serial.println();
  Serial.print("Connecting to ");
  if(NetChoice){Serial.println(ssid);}
  else{Serial.println(ssid2);}
  if(NetChoice){
    return wifi_connect(ssid,password);
  }
  else{
    return wifi_connect(ssid2,password2);
  }
}
void setup_wifi() {
  
  bool Network1 = true;
  bool connected = false;
  delay(10);
  
  while(true){
    connected = Connect(Network1);
    if(!connected){
      Network1 = false;
    }
    else{
      break;
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("NodeEmerge","node","node")) {
      Serial.println("connected");
     
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void setup( )
{
  // pinMode(AnemPin,INPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  dht.begin();
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }
}

float PPMConvert(int sensorValue, float R0){

  float RS_gas = 0;
  float ratio = 0;
  float sensor_volt = 0;
  sensor_volt = (float)sensorValue/4096*3.3;
  RS_gas = (3.3-sensor_volt)/sensor_volt;
  ratio = RS_gas/R0; // From https://www.teachmemicro.com/use-mq-7-carbon-monoxide-sensor/

  float x = 1538.46 * ratio;
  float ppm = pow(x,-1.709);
  Serial.print("PPM: ");
  Serial.println(ppm);
  return ppm;
}
void ReadGasSensors(){
  int gasValue = analogRead(MQ2Pin);
  int CMonoxideValue = analogRead(MQ7Pin);
  int AirQualityValue = analogRead(MQ135Pin);
  Serial.print("Gas: ");Serial.println(gasValue);
  gas = PPMConvert(gasValue, MQ2R0);
  Serial.print("CO: ");Serial.println(CMonoxideValue);
  carbonmonoxide = PPMConvert(CMonoxideValue, MQ7R0);
  Serial.print("Air: ");Serial.println(AirQualityValue);
  air = PPMConvert(AirQualityValue, MQ135R0);
 
 
 }

void ReadDHT(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {

  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {

    humidity = event.relative_humidity;

  }
}

void ReadBMP(){
   -
    temperature = bmp.readTemperature(); 
    
    pressure = bmp.readPressure();


    altitude = bmp.readAltitude();


}

void ReadWindSpeed(){
  int AnemRead= 0;
  int AnemReadAvg = 0;
  float windspeedms = 0;
  // Read Speed
  for (int i = 0; i<TotalAnemRead;i++){
  int AnemReadCheck = analogRead(AnemPin);
  AnemReadAvg+=AnemReadCheck;
    if(AnemReadCheck>AnemRead){
      AnemRead = AnemReadCheck;
    }
    delay(2);
  } 
  if(AnemReadAvg/TotalAnemRead < 90){ windspeedms = 0;}
  else{
  windspeedms = ((AnemReadAvg/TotalAnemRead))*1.2/142.857;}
  Serial.print("Wind Speed: ");
  Serial.print(AnemRead);
  Serial.print(" Wind Speed AVG: ");
  Serial.print(AnemReadAvg/TotalAnemRead);
  Serial.print(" Wind Speed M/S AVG: ");
  Serial.println(windspeedms);
  windspeed = windspeedms;
}
const char* ReturnJSON(){

  String ValString = "{temperature:"+String(temperature)+","+"pressure:"+String(pressure)+","+"windspeed:"+String(windspeed)+","+"humidity:"+String(humidity)+","+"airquality:"+String(air)+","+"gassensor:"+String(gas)+","+"carbonmonoxide:"+String(carbonmonoxide)+","+"altitude:"+String(altitude)+"}";

  return ValString.c_str();
  
  }

void loop( )
{
  if (!client.connected()) {
    reconnect();
  }
  
  // Read Speed
  ReadWindSpeed();
  // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  delay(300);

  //Read Gas Sensors
  ReadGasSensors();
  // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  delay(300);
  
  //Read DHT
  ReadDHT();
  // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  delay(300);

  //ReadBMP
  ReadBMP();
  // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  delay(300);

  // Serial.println("End Loop");

  client.publish("v1/devices/me/telemetry", ReturnJSON());
  client.loop();
}

