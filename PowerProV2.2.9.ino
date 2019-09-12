 #include <FS.h>               

#include <PubSubClient.h>
#include <ESP8266WiFi.h>          
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>             
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <JCCS5490.h>
#include <hw_timer.h>
#include <JCLib.h>

#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

//////////////////////////////////////////////////////
//#define TRUC_VERSION "LightProV2.2.7"
#define TRUC_VERSION "PowerProV2.2.9"
#define PowerPro
//////////////////////////////////////////////////////

JCCS5490 line(MCLK_default,5, 4);//Red boards
//JCCS5490 line(MCLK_default,4, 5);//Prototype boards

//================== WIFI VARs ==================
ESP8266WiFiMulti WiFiMulti;
CS5490 cs5490;
DIMMER dimmer;
FEEDBACK feedback;

// MQTT DEFAULT SETINGS
char mqtt_server[40] = "10.0.0.69";
char mqtt_port[6] = "1883";
const int charMax = sizeof(String(ESP.getChipId(), DEC));
char proDevice_token[charMax];


// FLAG FOR SAVING DATA
bool shouldSaveConfig = false;

//*********************  If defined *********************
//#define WIFI_RESET_SETTINGS
//#define SERIAL_DEBUG_ENABLE
#define CS5490_ENABLE


#ifdef PowerPro
String hostname("PowerPro1-");
#else
//String hostname("TestPro-");
String hostname("LightPro1-");

#endif
//********************************************************  
bool RUN_CS5490 = false;
bool CS5490 = false;


// DECLARE GPIO
int LT1 = 16;
int LT2 = 14;
int DOMODE = 12;
int FB_A = 13;
int FB_B = 3;

//int LT2 = 0;
int LTAUX1;
int lastState;

int boot = 0;
int SDBoot = 2;
int d15 = 15;

// Global Variables
unsigned long timeMe = 0;
unsigned long timeMe2 = 0;
int CS5490restarts = 0;
int CS5490TempCounter = 0;
unsigned int everyFiveDays = 0;
unsigned int resetTimer = 65000;//24 is 120 seconds; 65000 is 5 days

// Configuration that we'll store on disk
struct Config {
  long VGain = 5036613;
  long IGain = 4173953;
};
struct Power {
  int current = 15; 
  int voltage = 228;
};
struct Resets {
  int numberOfResets = 3; 
  bool powerStatus = false;
  bool newChip = false;
};

//Voltage & Current Calibration
int Imsb;// = 54;
int Imid;// = 219;
int Ilsb;// = 110;

const char *configFile1 = "/JCconfigFile1.txt";
const char *powerConfigFile1 = "/JCpowerConfigFile1.txt";  // <- SD library uses 8.3 filenames
const char *resetConfigFile1 = "/JCresetConfigFile1.txt";

Config config;
Power power;
Resets resets;


WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient client(espClient);




/***************************************************************************************
 SETUP Function -
/***************************************************************************************/
void setup() 
{

  #ifdef SERIAL_DEBUG_ENABLE
    Serial.begin(115200);
  #else
    pinMode(FB_B, FUNCTION_3);//
    pinMode(FB_B, INPUT_PULLUP);//
    //pinMode(FB_B, INPUT);//5424241; 5424787; 5422024
  #endif
  
  /* INIT GPIO */
  pinMode(LT1, OUTPUT);//D
  pinMode(LT2, OUTPUT);//D
  
  pinMode(DOMODE, INPUT_PULLUP);//
  pinMode(FB_A, INPUT_PULLUP);//
  //pinMode(FB_A, INPUT);//5424241; 5424787; 5422024

  /* BOOT GPIO */
  pinMode(boot, INPUT_PULLUP);//D0
  pinMode(SDBoot, INPUT_PULLUP);//D2
  //pinMode(d15, LOW);//D15



  /* Manual Override */ // ***Dont use this if device is already calibrated***
//  power.voltage = 231;
//  power.current = 15;
//  config.VGain = 5036613;
//  config.IGain = 4173953;
//  delay(250);
//  saveConfiguration(configFile1, config);
//  delay(250);
//  saveConfiguration_Current_Voltage(powerConfigFile1, power);
//  delay(250);

  /* ConfigFile */
  loadConfiguration(configFile1, config);
  delay(250);
  loadConfiguration_VI(powerConfigFile1, power);
  delay(250);
  loadConfiguration_Resets(resetConfigFile1, resets);
  delay(250);

  

  /* INIT CS5490 */
  #ifdef CS5490_ENABLE
    cs5490Setup();
    timeMe = millis();

    //Calibration at 2.5A / 15A
    Imsb = 12;
    Imid = 204;
    Ilsb = 205;
    
  #else
    //saveConfiguration(configFile1, config);

    //Calibration at 2.5A / 3A
    Imsb = 64;
    Imid = 0;
    Ilsb = 0;
  #endif
//----------
  #ifdef PowerPro
  WiFi.hostname("PowerPro-" + String(ESP.getChipId(), DEC));
  if(resets.powerStatus)
  {
    digitalWrite(LT1, LOW);
    delay(500);
    digitalWrite(LT2, HIGH);
  }

  #else
  WiFi.hostname("LightPro1-" + String(ESP.getChipId(), DEC));
  /* INIT INTERRUPTS */
  //attachInterrupt(DOMODE, zcDetectISR1, RISING);//PHASE CHOPPER - DIMMER
  //attachInterrupt(digitalPinToInterrupt(FB_A), switchFeedback1, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(FB_B), switchFeedback2, CHANGE);
  
  /* INIT HARDWARE TIMER */
  //hw_timer_init(NMI_SOURCE, 0);
  //hw_timer_set_func(dimTimerISR1);
  #endif

  delay(15000);

  /* WiFi Setup */
  wifiSetup();
  

  /* INIT TEST FUNC */
  ArduinoOTA.onStart([](){stopTimerMacro();});
    
//  ArduinoOTA.onError([](ota_error_t error) {
//    (void)error;
//    ESP.restart();
//  });


}


/***************************************************************************************
 LOOP Function -
/***************************************************************************************/
void loop() 
{
  // Handle OTA server.
  ArduinoOTA.handle();
  yield();
  if(!client.connected())
  {
  //Serial.println("======== !client.connected ========");
    client.disconnect();
    reconnect();
  }
  client.loop();

  // Initiate Start-up delay Sequence for MQTT
  if(feedback.MQTTinit)
  {
    feedback.MQTTinit = false;
    feedback.MQTTreconnect = true; 
    timeMe = millis();
  }
  // Initiate Start-up delay Sequence for MQTT
  if(millis() > (timeMe + 5000) && feedback.MQTTreconnect)
  {
    feedback.MQTTreconnect = false;
    feedback.MQTTreconnectFinished = true;
    timeMe = millis();
  }
  // Initiate Start-up delay Sequence for MQTT
  if(millis() > (timeMe + 1000) && feedback.MQTTreconnectFinished)
  { 
    timeMe = millis();
    powerChip();
    checkStatus();
  }
  // Initiate Start-up delay Sequence for MQTT
  if(millis() > (timeMe2 + 5000) && feedback.MQTTreconnectFinished)
  { 
    timeMe2 = millis();
    pubSwitchStatus();
    everyFiveDays++;
    //Serial.println("WiFi.localIP(): " + String(WiFi.localIP()));
    //Serial.println("WiFi.SSID(): " + String(WiFi.SSID()));
    //Serial.println("Hostname: " + hostname);
    //Serial.println("ESP.getChipId(): " + String(ESP.getChipId(), DEC));
    Serial.println("everyFiveDays: " + String(everyFiveDays));

    // CHECK FOR NEW SOFTWARE UPDATES EVERY 5 DAYS
    if(everyFiveDays >= resetTimer)//65000 is 5 days, 24 is 120 secs
    {
      everyFiveDays = 0;
      Serial.println("TRUC_VERSION Try 1: " + String(TRUC_VERSION));
      t_httpUpdate_return ret1 = ESPhttpUpdate.update("http://10.0.0.69:1880/DeleteDownload",TRUC_VERSION);
      t_httpUpdate_return ret2 = ESPhttpUpdate.update("http://10.0.0.69:1880/firmwareUpdate",TRUC_VERSION);
      //t_httpUpdate_return  ret = ESPhttpUpdate.update("https://server/file.bin", "", "fingerprint");
      Serial.println("TRUC_VERSION Try 2: " + String(TRUC_VERSION));
      switch (ret2) 
      {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
    
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
    
        case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_OK");
          ESP.restart();
          break;
      }  
    }  
  }

  // READ MANUAL SWITCH INPUTS AND SWITCH ACCORDINGLY
  bool lightStatus;
  if((digitalRead(FB_A) == HIGH) && (feedback.mode11 == true))
  {
    digitalWrite(LT1, !feedback.lastStateLt1);
    client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt1, DEC).c_str());
    feedback.lastStateLt1 = !feedback.lastStateLt1;
    feedback.mode11 = false;
    delay(500);
  }
  if((digitalRead(FB_A) == LOW) && (feedback.mode11 == false))
  {
    digitalWrite(LT1, !feedback.lastStateLt1);
    client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt1, DEC).c_str());
    feedback.lastStateLt1 = !feedback.lastStateLt1;
    feedback.mode11 = true;
    delay(500);
  }
  if((digitalRead(FB_B) == HIGH) && (feedback.mode22 == true))
  {
    digitalWrite(LT2, !feedback.lastStateLt2);
    client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt2, DEC).c_str());
    feedback.lastStateLt2 = !feedback.lastStateLt2;
    feedback.mode22 = false;
    delay(500);
  }
  if((digitalRead(FB_B) == LOW) && (feedback.mode22 == false))
  {
    digitalWrite(LT2, !feedback.lastStateLt2);
    client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt2, DEC).c_str());
    feedback.lastStateLt2 = !feedback.lastStateLt2;
    feedback.mode22 = true;
    delay(500);
  }
}

void saveConfiguration(const char *filename, Config &config){
  
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["VGain"] = config.VGain;
    json["IGain"] = config.IGain;
    File configFile = SPIFFS.open(filename, "w");
    if (!configFile) {
      //line.cSerial1->println("failed to open config file for writing");
    }
    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save

}

void saveConfiguration_Current_Voltage(const char *filename, Power &power){
  
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["voltage"] = power.voltage;
    json["current"] = power.current;
    File configFile = SPIFFS.open(filename, "w");
    if (!configFile) {
      //line.cSerial1->println("failed to open config file for writing");
    }
    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save

}

void saveConfiguration_Resets(const char *filename, Resets &resets){
  
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["numberOfResets"] = resets.numberOfResets;
    json["powerStatus"] = resets.powerStatus;
    json["newChip"] = resets.newChip;
    File configFile = SPIFFS.open(filename, "w");
    if (!configFile) {
      //line.cSerial1->println("failed to open config file for writing");
    }
    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save

}

void loadConfiguration(const char *filename, Config &config){
//read configuration from FS json
  if (SPIFFS.begin()) 
  {
    if (SPIFFS.exists(filename)) 
    {
      //file exists, reading and loading
      File configFile = SPIFFS.open(filename, "r");
      if (configFile) {
        //line.cSerial1->println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        //json.printTo(Serial);
        if (json.success()) 
        {
          config.VGain = json["VGain"];
          config.IGain = json["IGain"];
        } 
        else 
        {
          //line.cSerial1->println("failed to load json config");
        }
        configFile.close();
      }
    }
  } 
  else 
  {
    //line.cSerial1->println("failed to mount FS");
  }
  //end read
}

void loadConfiguration_VI(const char *filename, Power &power){
//read configuration from FS json
  if (SPIFFS.begin()) 
  {
    if (SPIFFS.exists(filename)) 
    {
      //file exists, reading and loading
      File configFile = SPIFFS.open(filename, "r");
      if (configFile) {
        //line.cSerial1->println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        //json.printTo(Serial);
        if (json.success()) 
        {
          power.voltage = json["voltage"];
          power.current = json["current"];
        } 
        else 
        {
          //line.cSerial1->println("failed to load json config");
        }
        configFile.close();
      }
    }
  } 
  else 
  {
    //line.cSerial1->println("failed to mount FS");
  }
  //end read
}

void loadConfiguration_Resets(const char *filename, Resets &resets){
//read configuration from FS json
  if (SPIFFS.begin()) 
  {
    if (SPIFFS.exists(filename)) 
    {
      //file exists, reading and loading
      File configFile = SPIFFS.open(filename, "r");
      if (configFile) {
        //line.cSerial1->println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        //json.printTo(Serial);
        if (json.success()) 
        {
          resets.numberOfResets = json["numberOfResets"];
          resets.powerStatus = json["powerStatus"];
          resets.newChip = json["newChip"];
        } 
        else 
        {
          //line.cSerial1->println("failed to load json config");
        }
        configFile.close();
      }
    }
  } 
  else 
  {
    //line.cSerial1->println("failed to mount FS");
  }
  //end read
}


//void ICACHE_FLASH_ATTR timer1Stop(void)
//{
//    ETS_FRC1_INTR_DISABLE();
//    TM1_EDGE_INT_DISABLE();
//}

void stopTimerMacro()
{
  //timer1Stop();
}

//void ICACHE_FLASH_ATTR starTimerMacro()
//{
//  hw_timer_init(NMI_SOURCE, 0);
//  hw_timer_set_func(dimTimerISR1);
//  TM1_EDGE_INT_ENABLE();
//  ETS_FRC1_INTR_ENABLE();
//}

// func12
void Gaincalibration()
{
  client.publish("config.IGain", "Starting Configuration!");
  
  client.disconnect();
  delay(500);
  //Serial.println("CS5490 calibration: Reset IGain to default");
  client.publish("config.IGain", "CS5490 calibration: Reset IGain to default");
  line.write(16, 33, 64, 0, 0);//Reset IGain to default
  delay(250);
  //Serial.println("CS5490 calibration: Reset VGain to default");
  client.publish("config.IGain", "CS5490 calibration: Reset VGain to default");
  line.write(16, 35, 64, 0, 0);//Reset VGain to default
  delay(250);
  //Serial.println("CS5490 calibration: Set Tsettle to 2000ms");
  client.publish("config.IGain", "CS5490 calibration: Set Tsettle to 2000ms");
  line.write(16, 57, 0, 7, 208);//Set Tsettle to 2000ms
  delay(250);
  //Serial.println("CS5490 calibration: Scale Register 2.5A");
  client.publish("config.IGain", "CS5490 calibration: Scale Register");
  line.write(18,63, Imsb, Imid, Ilsb);//Scale Register Ical/Imax
  delay(250);
  //Serial.println("CS5490 calibration: Gain V & I");
  client.publish("config.IGain", "CS5490 calibration: Gain V & I");
  line.instruct(61);//Calibrate - Gain V & I
  delay(250);
  //Serial.println("CS5490 calibration: Clear DRDY & CRDY in Status Register");
  client.publish("config.IGain", "CS5490 calibration: Clear DRDY & CRDY in Status Register");
  line.write(0, 23, 255, 0, 0);//Clear DRDY & CRDY in Status Register
  delay(8000);
  //Serial.println("CS5490 calibration: Reset Tsettle to 30ms default");
  client.publish("config.IGain", "CS5490 calibration: Reset Tsettle to 30ms default");
  line.write(16, 57, 0, 0, 30);//Reset Tsettle to 30ms default
  delay(250);
  //Serial.println("CS5490 calibration: Halt Conversions");
  client.publish("config.IGain", "CS5490 calibration: Halt Conversions");
  line.haltConv();//CS5490 - Halt Conversions
  delay(250);
  //Serial.println("CS5490 calibration: Start Continuious Conversions");
  client.publish("config.IGain", "CS5490 calibration: Start Continuious Conversions");
  line.contConv();//CS5490 - Start Continuious Conversions
  delay(5000);
  
  //Serial.println("CS5490 calibration: config.VGain = line.readReg(16,35);");
  client.publish("config.IGain", "CS5490 calibration: config.VGain = line.readReg(16,35);");
  config.VGain = line.readReg(16,35);
  config.IGain = line.readReg(16,33);
  
  //Serial.println("CS5490 calibration: saveConfiguration(filename, config)");
  client.publish("config.IGain", "CS5490 calibration: saveConfiguration(filename, config)!");
  saveConfiguration(configFile1, config);
  saveConfiguration_Current_Voltage(powerConfigFile1, power);
  delay(500);
  reconnect();
  delay(500);
  
  //Serial.println("CS5490 calibration: MQTT Confirm");
  client.publish("config.IGain", "CS5490 calibration: MQTT Confirm!");
  client.publish("config.VGain", String(config.VGain, DEC).c_str());
  client.publish("config.IGain", String(config.IGain, DEC).c_str());
  
  delay(1500);
}

// func11
//void switchFeedback1() 
//{
//  if(!feedback.mode1)
//  {
//    feedback.mode1 = true;
//    if((long)(micros() - feedback.last_micros1) >= feedback.debouncing_time1 * 1500) 
//    {
//      Interrupt1();
//      feedback.last_micros1 = micros();
//    }
//    feedback.mode1 = false;
//  }
//}
//
//// func10
//void Interrupt1() 
//{
//  feedback.lastStateLt1 = digitalRead(LT1);
//  digitalWrite(LT1, !feedback.lastStateLt1);
//  client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt1, DEC).c_str());
//}
//
//// func11
//void switchFeedback2() 
//{
//  if(!feedback.mode1)
//  {
//    feedback.mode1 = true;
//    if((long)(micros() - feedback.last_micros2) >= feedback.debouncing_time2 * 1500) 
//    {
//      Interrupt2();
//      feedback.last_micros2 = micros();
//    }
//    feedback.mode1 = false;
//  }
//}
//
//// func10
//void Interrupt2() 
//{
//  feedback.lastStateLt2 = digitalRead(LT2);
//  digitalWrite(LT2, !feedback.lastStateLt2);
//  client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt2, DEC).c_str());
//}



// func7
void saveConfigCallback() 
{         
  //line.cSerial1->println("Should save config");
  shouldSaveConfig = true;
}

// func6
void pubSwitchStatus()
{
//    client.publish(("ProPower-ESP/volts/" + (String)proDevice_token).c_str(), ((String)cs5490.data.volts).c_str());
//    delay(50);
//    client.publish(("ProPower-ESP/amps/" + (String)proDevice_token).c_str(), ((String)cs5490.data.amps).c_str());
//    delay(50);
//    client.publish(("ProPower-ESP/kwh/" + (String)proDevice_token).c_str(), ((String)String(cs5490.data.kwh, 12)).c_str());
//    delay(50);
//    client.publish(("ProPower-ESP/temp/" + (String)proDevice_token).c_str(), ((String)String(cs5490.data.temp)).c_str());
//    delay(50);
    client.publish(("ProPower-ESP/Device/" + (String)proDevice_token).c_str(), 
    String(String(cs5490.data.volts) + "," + String(cs5490.data.amps) + "," +  String(cs5490.data.kwh, 12).c_str() + "," +  String(TRUC_VERSION)).c_str());
}

void checkStatus()
{
    client.publish(("ProPower-ESP/watts/" + (String)proDevice_token).c_str(), ((String)cs5490.data.watts).c_str());
    delay(50);
    
  #ifdef PowerPro
    if(digitalRead(LT2) == HIGH)
    {
      client.publish(("ProPower-ESP/STAT/RELAY/" + (String)proDevice_token).c_str(), String(1, DEC).c_str());
    }
    else
    {
      client.publish(("ProPower-ESP/STAT/RELAY/" + (String)proDevice_token).c_str(), String(0, DEC).c_str());
    }
      
  #else
      client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), ((String)digitalRead(LT1)).c_str());
      delay(50);
      client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), ((String)digitalRead(LT2)).c_str());
      delay(50);
  #endif
//  Serial.println("Test String - millis(): " + String(millis()));
}

// func5
void MQTTcallback(char* topic, byte* payload, unsigned int length) 
{
  String strMsg;
  String strTopic;
  //Serial.print("Message arrived ========================\n [");
  //Serial.print(topic);
  //Serial.print("] ");
  //===== MESSAGE =====
  for (int i=0;i<length;i++) 
  {
    //Serial.print((char)payload[i]);
    strMsg += (char)payload[i];
  }
  //==================
  //======TOPIC ======
  for(int i=0;i<strlen(topic);i++)
  {
    strTopic += topic[i];
  }
  //line.cSerial1->println(strTopic);
  //==================

  #ifdef PowerPro
  ///////ESP-POWER-RELAY
  if(strTopic == ("ProPower-ESP/CMD/RELAY/" + (String)proDevice_token).c_str())
  {
    if(strMsg == "1")
    {
      resets.powerStatus = true;
      saveConfiguration_Resets(resetConfigFile1, resets);
      digitalWrite(LT1, LOW);
      delay(500);
      digitalWrite(LT2, HIGH);
//      delay(500);
//      digitalWrite(LT2, LOW);
      lastState = 1;
      delay(50);
      client.publish(("ProPower-ESP/STAT/RELAY/" + (String)proDevice_token).c_str(), ((String)lastState).c_str());
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
    else if(strMsg == "0")
    {
      resets.powerStatus = false;
      saveConfiguration_Resets(resetConfigFile1, resets);
      digitalWrite(LT2, LOW);
      delay(500);
      digitalWrite(LT1, HIGH);
//      delay(500);
//      digitalWrite(LT1, LOW);
      lastState = 0;
      delay(50);
      client.publish(("ProPower-ESP/STAT/RELAY/" + (String)proDevice_token).c_str(), ((String)lastState).c_str());
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
  }
  else if(strTopic == ("ProPower-ESP/REQUEST/" + (String)proDevice_token).c_str())
  {
    client.publish(("ProPower-ESP/STAT/RELAY/" + (String)proDevice_token).c_str(), ((String)lastState).c_str());
    delay(50);
    pubSwitchStatus();
  }
  else if(strTopic == ("ProPower-ESP/CMD/VOLTAGE/" + (String)proDevice_token).c_str())
  {power.voltage = strMsg.toInt();//}
  client.publish("config.IGain",  String(power.voltage).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/CURRENT/" + (String)proDevice_token).c_str())
  {power.current = strMsg.toInt();//}
  client.publish("config.IGain",  String(power.current).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Imsb/" + (String)proDevice_token).c_str())
  {Imsb = strMsg.toInt();//}
  client.publish("config.IGain",  String(Imsb).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Imid/" + (String)proDevice_token).c_str())
  {Imid = strMsg.toInt();//}
  client.publish("config.IGain",  String(Imid).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Ilsb/" + (String)proDevice_token).c_str())
  {Ilsb = strMsg.toInt();//}
  client.publish("config.IGain",  String(Ilsb).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/CALI/" + (String)proDevice_token).c_str())
  {
    if(strMsg == "1") 
    {
      Gaincalibration();  
    }
  }
  strTopic = "";
  strMsg = "";


  #else
  ///////ESP-LIGHTING
  if(strTopic == ("ProPower-ESP/CMD/LTA/" + (String)proDevice_token).c_str())
    {
      if(strMsg == "1")
    {
      digitalWrite(LT1, HIGH);
      delay(50);
      client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt1, DEC).c_str());
      feedback.lastStateLt1 = !feedback.lastStateLt1;
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
    else if(strMsg == "0")
    {
      digitalWrite(LT1, LOW);
      delay(50);
      client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt1, DEC).c_str());
      feedback.lastStateLt1 = !feedback.lastStateLt1;
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
  }
  else if(strTopic == ("ProPower-ESP/CMD/LTB/" + (String)proDevice_token).c_str())
  {
    if(strMsg == "1")
    {
      digitalWrite(LT2, HIGH);
      delay(50);
      client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt2, DEC).c_str());
      feedback.lastStateLt2 = !feedback.lastStateLt2;
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
    else if(strMsg == "0")
    {
      digitalWrite(LT2, LOW);
      delay(50);
      client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), String(!feedback.lastStateLt2, DEC).c_str());
      feedback.lastStateLt2 = !feedback.lastStateLt2;
      delay(3000);
      powerChip();
      pubSwitchStatus();
    }
  }
  else if(strTopic == ("ProPower-ESP/REQUEST/" + (String)proDevice_token).c_str())
  {
    client.publish(("ProPower-ESP/STAT/LTA/" + (String)proDevice_token).c_str(), ((String)digitalRead(LT1)).c_str());
    delay(50);
    client.publish(("ProPower-ESP/STAT/LTB/" + (String)proDevice_token).c_str(), ((String)digitalRead(LT2)).c_str());
    delay(50);
    pubSwitchStatus();
  }
  else if(strTopic == ("ProPower-ESP/CMD/FADE1/" + (String)proDevice_token).c_str())
  {
    
    dimmer.timer1 = strMsg.toInt(); 
    client.publish("config.IGain", String(dimmer.timer1).c_str());
    //Serial.println("ProPower-ESP/CMD/FADE1/" + String(dimmer.timer1));
    RUN_CS5490 = true;
    //tarBrightness1 = strMsg.toInt(); 
  }
  else if(strTopic == ("ProPower-ESP/CMD/FADE2/" + (String)proDevice_token).c_str())
  {
    dimmer.timer2 = strMsg.toInt();
    client.publish("config.IGain",  String(dimmer.timer2).c_str());
    //Serial.println("ProPower-ESP/CMD/FADE2/" + String(dimmer.timer2));
    //tarBrightness2 = strMsg.toInt();   
  }
  else if(strTopic == ("ProPower-ESP/CMD/VOLTAGE/" + (String)proDevice_token).c_str())
  {power.voltage = strMsg.toInt();//}
  client.publish("config.IGain",  String(power.voltage).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/CURRENT/" + (String)proDevice_token).c_str())
  {power.current = strMsg.toInt();//}
  client.publish("config.IGain",  String(power.current).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Imsb/" + (String)proDevice_token).c_str())
  {Imsb = strMsg.toInt();//}
  client.publish("config.IGain",  String(Imsb).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Imid/" + (String)proDevice_token).c_str())
  {Imid = strMsg.toInt();//}
  client.publish("config.IGain",  String(Imid).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/Ilsb/" + (String)proDevice_token).c_str())
  {Ilsb = strMsg.toInt();//}
  client.publish("config.IGain",  String(Ilsb).c_str());}
  else if(strTopic == ("ProPower-ESP/CMD/CALI/" + (String)proDevice_token).c_str())
  {
    if(strMsg == "1") 
    {
      Gaincalibration();  
    }
  }
  strTopic = "";
  strMsg = "";
  #endif
}

// func4
void powerChip()
{
  float val = line.readReg(16, 7);
  //cs5490.data.volts = val / (0.6 * pow(2,24)) * 230; 
  cs5490.data.volts = val / (0.6 * pow(2,24)) * power.voltage;
  val = line.readReg(16, 6);
  cs5490.data.amps = val / (0.6 * pow(2,24)) * power.current; 
  cs5490.data.watts = cs5490.data.amps * cs5490.data.volts;
  cs5490.data.kwh += (cs5490.data.watts / 1000) / 3600;
  cs5490.data.amps *= 1000;


  #ifdef PowerPro
    if(cs5490.data.amps > 15500)
    {
      digitalWrite(LT2, LOW);
      delay(500);
      digitalWrite(LT1, HIGH);
    }
  #else
    if(cs5490.data.amps > 20000)
    {
      digitalWrite(LT1, LOW);
      delay(100);
      digitalWrite(LT2, LOW);
    }
  #endif
  

  //Temperature
  //line.write(0, 23, 0, 0, 32);//Clear Temp bit in Status Register
  //val = line.readReg(16, 27);
  //val = line.readReg(0, 23);//status0
  //float x = pow(2,16);
  //cs5490.data.temp = val / x;
  
  line.data[0] = 0;
  line.data[1] = 0;
  line.data[2] = 0;

//  CS5490TempCounter++;

  #ifdef CS5490_ENABLE
    if(CS5490restarts == 10)
    {
      CS5490restarts = 0;
      ESP.restart();
    }
  
  //  if(CS5490TempCounter == 30)
  //  {
  //    CS5490TempCounter = 0;
  //    line.reset();
  //  }
  
    if(cs5490.data.watts == 0)
    {
      client.disconnect();
      CS5490restarts++;
      cs5490Setup();
      reconnect();
    }
  #endif

}

// func3
void cs5490Setup()
{
  /////CS5490 Initialise
  delay(500);
  //Serial.println("CS5490 Reset");
  line.reset();//CS5490 - Software Reset

  delay(500);
  //Serial.println("CS5490 Init: Baud Rate Begin(600)");
  line.begin(600);//600

  //delay(500);
  //Serial.println("CS5490 Init: Baud Rate & Enable Checksum");
  //line.write(0, 7, 2, 4, 205);//Set baud rate to (9600)
  //line.write(0, 7, 0, 19, 52);//Set baud rate to (38400)checksum
  line.write(0, 7, 2, 19, 52);//Set baud rate to (38400)no checksum
  //line.write(0, 7, 2, 28, 205);//Set baud rate to (57600)
  //line.write(0, 7, 2, 57, 154);//Set baud rate to (115200)
  
  //delay(500);
  //Serial.println("CS5490 Init: Baud Rate Begin(38400)");
  line.begin(38400);
  //line.begin(57600);
  //line.begin(115200);
  
  delay(500);
  //Serial.println("CS5490 Init: ContConv()");
  line.contConv();//CS5490 - Start Continuious Conversions

  delay(500);
  //Serial.println("CS5490 Init: V Zero Crossing");
  line.write(0, 1, 0, 0, 11);//V Zero Crossing

  delay(500);
  //Temperature Gain
  //line.write(16, 54, 120, 120, 120);//Clear Temp bit in Status Register

  delay(500);
  //Temperature Offset
  //line.write(16, 55, 120, 120, 120);//Clear Temp bit in Status Register

  delay(500);
  //Serial.println("CS5490 Init: Disable internal crystal oscillator");
  line.write(0, 0, 0, 0, 4);//Disable internal crystal oscillator

  delay(500);
  //Serial.println("CS5490 Init: VGain Registers");
  line.data[0] = config.VGain;//LSB
  line.data[1] = config.VGain>>8;//MID
  line.data[2] = config.VGain>>16;//MSB
  line.write(16, 35, line.data[2], line.data[1], line.data[0]);//Set VGain Register

  delay(500);
  //Serial.println("CS5490 Init: IGain Registers");
  line.data[0] = config.IGain;//LSB
  line.data[1] = config.IGain>>8;//MID
  line.data[2] = config.IGain>>16;//MSB
  line.write(16, 33, line.data[2], line.data[1], line.data[0]);//Set IGain Register

  delay(500);
  //Serial.println("CS5490 Init: ContConv()");
  line.contConv();
  CS5490 = true;
  //Serial.println("CS5490 Init: CS5490 = true");
 
}

// func2
void wifiSetup()
{
  String(ESP.getChipId(), DEC).toCharArray(proDevice_token, charMax);

  #ifdef PowerPro
    hostname = "PowerPro-" + String(ESP.getChipId(), DEC);
  #else
    hostname = "LightPro-" + String(ESP.getChipId(), DEC);
    //hostname = "TestPro-" + String(ESP.getChipId(), DEC);
  #endif

  //clean FS, for testing
  #ifdef WIFI_RESET_SETTINGS
  SPIFFS.format();
  #endif

  //read configuration from FS json
  //line.cSerial1->println("mounting FS...");

  if (SPIFFS.begin()) {
    //line.cSerial1->println("mounted file system");
    if (SPIFFS.exists("/config.json")) 
    {
      //file exists, reading and loading
      //line.cSerial1->println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        //line.cSerial1->println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        //json.printTo(Serial);
        if (json.success()) {
          //line.cSerial1->println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(proDevice_token, json["proDevice_token"]);

        } 
        else 
        {
          //line.cSerial1->println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    //line.cSerial1->println("failed to mount FS");
  }
  //end read



  // Set Hostname.
  //hostname += String(ESP.getChipId(), DEC);
  //WiFi.hostname("PowerPro1-" + String(ESP.getChipId(), DEC));
  ArduinoOTA.setHostname((const char *)hostname.c_str());


  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_proDevice_token("token", "proDevice token", proDevice_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_proDevice_token);

  //reset settings - for testing
  #ifdef WIFI_RESET_SETTINGS
  wifiManager.resetSettings();
  #endif
  
  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);


  // DELAY AP MODE FOR (2.5 MINUTES)
  int a = 0;
  while(a <= 150 && WiFi.status() != WL_CONNECTED)
  {
    a++;
    //Serial.println("Get Fucked you WiFi piece of shit!!!!!!!!!!!!!!!!!!!!!!!!!!11");
    delay(1000);
  }
  

  
  #ifdef PowerPro
    if (!wifiManager.autoConnect(("PowerPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
      //line.cSerial1->println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  #else
    if (!wifiManager.autoConnect(("LightPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
    //if (!wifiManager.autoConnect(("TestPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
      //line.cSerial1->println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  #endif
   
  
  

  //if you get here you have connected to the WiFi
  //line.cSerial1->println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(proDevice_token, custom_proDevice_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    //line.cSerial1->println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["proDevice_token"] = proDevice_token;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      //line.cSerial1->println("failed to open config file for writing");
    }
    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  // Start OTA server.
  ArduinoOTA.begin();
  
  // Start MQTT server.
  client.setServer(mqtt_server, ((String)mqtt_port).toInt());
  client.setCallback(MQTTcallback);
}

// func1
void reconnect() 
{
  // Loop until we're reconnected

  int reconnectTries = 0;
  
  while (!client.connected())
  {    
    //Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //if you MQTT broker has clientID,username and password
    //please change following line to    if (client.connect(clientId,userName,passWord))
    if (client.connect(clientId.c_str()))
    {
      //Serial.println("connected");
      // Once connected, publish an announcement...
      // ... and resubscribe
      #ifdef PowerPro
      client.subscribe(("ProPower-ESP/CMD/RELAY/" + (String)proDevice_token).c_str());
      #else
      client.subscribe(("ProPower-ESP/CMD/LTA/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/LTB/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/FADE1/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/FADE2/" + (String)proDevice_token).c_str());
      #endif
      client.subscribe(("ProPower-ESP/CMD/VOLTAGE/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/CURRENT/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/Imsb/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/Imid/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/Ilsb/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/CMD/CALI/" + (String)proDevice_token).c_str());
      client.subscribe(("ProPower-ESP/REQUEST/" + (String)proDevice_token).c_str());

      feedback.MQTTinit = true;
      resets.numberOfResets = 0;
      reconnectTries = 0;
      saveConfiguration_Resets(resetConfigFile1, resets);
      
      
    }
    else 
    {
      /* Retry reconnect ESP Client */
      loadConfiguration_Resets(resetConfigFile1, resets);
      if(reconnectTries >= 30 && resets.numberOfResets < 3)
      {
        resets.numberOfResets++;
        saveConfiguration_Resets(resetConfigFile1, resets);
        //Serial.println("resets.numberOfResets: " + String(resets.numberOfResets));
        ESP.reset();
      }
      
      feedback.MQTTreconnectFinished = false;
      Serial.print("failed, rc=");
      Serial.println(client.state());
      Serial.println("mqtt_server: " + String(mqtt_server));
      Serial.println("mqtt_port: " + String(mqtt_port));
      Serial.println("proDevice_token " + String(proDevice_token));
      Serial.println("WiFi.localIP(): " + String(WiFi.localIP()));
      Serial.println("WiFi.SSID(): " + String(WiFi.SSID()));
      Serial.println("Hostname: " + hostname);
      // Wait 6 seconds before retrying
      Serial.println("6000ms Start Delay");
      delay(6000);
      Serial.println("6000ms Finish Delay");
      Serial.print("reconnectTries: ");
      Serial.println(reconnectTries);
      Serial.print("numberOfResets: ");
      Serial.println(resets.numberOfResets);
      reconnectTries++;
      if(reconnectTries > 30 && resets.numberOfResets >= 3)
      {
        resets.numberOfResets = 0;
        saveConfiguration_Resets(resetConfigFile1, resets);
        
        #ifdef PowerPro
        if (!wifiManager.autoConnect(("PowerPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
          //line.cSerial1->println("failed to connect and hit timeout");
          delay(3000);
          //reset and try again, or maybe put it to deep sleep
          ESP.reset();
          delay(5000);
        }
        #else
        if (!wifiManager.autoConnect(("LightPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
        //if (!wifiManager.autoConnect(("TestPro-" + String(ESP.getChipId(), DEC)).c_str(), "password")) {
          //line.cSerial1->println("failed to connect and hit timeout");
          delay(3000);
          //reset and try again, or maybe put it to deep sleep
          ESP.reset();
          delay(5000);
        }
        #endif
      }
//      else
//      {
//        ESP.reset();
//      }
    }
  }
}
