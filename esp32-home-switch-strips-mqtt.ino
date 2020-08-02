#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

#include "FastLED.h" 
#include "arduinoFFT.h"

#include "_authentification.h"  /* credentials for WIFI and mqtt. Located in libraries folder */


/*****************************************************************************************/
/*                                    GENERAL DEFINE                                     */
/*****************************************************************************************/
#define TRUE  1
#define FALSE 0

#define STATE_OFF           0
#define STATE_ON            1

/*****************************************************************************************/
/*                                    PROJECT DEFINE                                     */
/*****************************************************************************************/

#define MQTT_PAYLOAD_MAX 120

/* Receive topics */
#define TOPIC_STRIP_1_PROGRAM_1_SET_ON        "wz/strip_1/program_1/set/on"
#define TOPIC_STRIP_1_PROGRAM_2_SET_ON        "wz/strip_1/program_2/set/on"
#define TOPIC_STRIP_1_PROGRAM_3_SET_ON        "wz/strip_1/program_3/set/on"


/* Send topics */
#define TOPIC_STRIP_1_PROGRAM_1_GET_ON        "wz/strip_1/program_2/get/on"

/* LED STRIPS */
#define NUM_LEDS 100
#define DATA_PIN 26
#define DATA_MIC_INPUT A0

/* FFT */
#define SAMPLES 128              /* Must be a power of 2 */
#define SAMPLING_FREQUENCY 5000  /* Hz, must be less than 10000 due to ADC */


/*****************************************************************************************/
/*                                     TYPEDEF ENUM                                      */
/*****************************************************************************************/
typedef enum 
{
  STRIP_PROGRAM_OFF = 0,
  STRIP_PROGRAM_01,
  STRIP_PROGRAM_02,
  STRIP_PROGRAM_03
}T_STRIP_PROGRAM;


/*****************************************************************************************/
/*                                   TYPEDEF STRUCT                                      */
/*****************************************************************************************/
struct HSV 
{
  int hue;
  int sat;
  int val;
};

/*****************************************************************************************/
/*                                         VARIABLES                                     */
/*****************************************************************************************/
/* create an instance of WiFiClientSecure */
WiFiClient espClient;
PubSubClient client(espClient);


int mqttRetryAttempt = 0;
int wifiRetryAttempt = 0;

                 
long lastMsg = 0;

T_STRIP_PROGRAM StripProgram = STRIP_PROGRAM_OFF;

/* Strip Program 1 (using Microphone with FFT) */
//volume
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
//smoothing
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = DATA_MIC_INPUT;
int f = 0;
int val = 175;
CRGB leds[NUM_LEDS];
//fft
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
HSV colorLED[NUM_LEDS];
HSV black;
/* END Strip Program 1 (using Microphone with FFT) */

/* Strip Program 2 (color) */
long Color_Val_R;
long Color_Val_G;
long Color_Val_B;
HSV color_format;
/* END Strip Program 2 (color) */

static void receivedCallback(char* topic, byte* payload, unsigned int length);
static void mqttconnect(void);

#define DEBUG_MQTT_RECEIVER   Serial.print("Message received: ");  \
                              Serial.print(topic); \
                              Serial.print("\t"); \
                              Serial.print("payload: "); \
                              Serial.println(PayloadString);

/**************************************************************************************************
Function: ArduinoOta_Init()
Argument: void
return: void
**************************************************************************************************/
void ArduinoOta_Init(void)
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32-home-switch-strips-mqtt");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}                              
/*************************************************************************************************/
/**************************************************************************************************
Function: setup()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void setup()
{
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("################################");
  Serial.println("# Program Home-Strips v0.2     #");
  Serial.println("################################");
  Serial.println(__FILE__);
  Serial.println(" ");
  Serial.println("Starting ...");
  Serial.println(" ");


  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
    wifiRetryAttempt++;
    if (wifiRetryAttempt > 5) 
    {
      Serial.println("Restarting!");
      ESP.restart();
    }
  }

  ArduinoOta_Init();
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("IP address of server: ");
  Serial.println(serverHostname);
  /* set SSL/TLS certificate */
  /* configure the MQTT server with IPaddress and port */
  client.setServer(serverHostname, 1883);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);
  
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  //fft
  randomSeed(98155);
  //smoothing
  for (int thisReading = 0; thisReading < numReadings; thisReading++) 
  {
    readings[thisReading] = 0;
  }
  //fft
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  delay(2000);
  black.hue = 0;
  black.sat = 0;
  black.val = 0;
    
  Serial.println("Setup finished ... ");
}

/*************************************************************************************************/
/**************************************************************************************************
Function: loop()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void loop() 
{

  ArduinoOTA.handle();
  
  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
    mqttconnect();
  }
  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  client.loop();
 
  Strip_Program_Cyclic();

  /* we increase counter every 5 secs we count until 5 secs reached to avoid blocking program if using delay()*/
  long now = millis();
  
  /* calling every 5 sec. */
  if (now - lastMsg > 5000)
  {
    /* store timer value */
    lastMsg = now;

    
    /********************************************************************************************/
    /************************      HANDLING OF Send MQTT TOPICS     *****************************/ 
    /********************************************************************************************/
    char data[MQTT_PAYLOAD_MAX];
    String json; 
       
  
    char temp[8];
    char humidity[8];
    //dtostrf(Dht.heat_index,  6, 2, temp);
    //dtostrf(Dht.humidity, 6, 2, humidity);
    json = "{\"temperature\":" + String(temp) + ",\"humidity\":" + String(humidity) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_STRIP_1_PROGRAM_1_GET_ON, data, false);

  }

}

/**************************************************************************************************
Function: receivedCallback()
Argument: char* topic ; received topic
          byte* payload ; received payload
          unsigned int length ; received length
return: void
**************************************************************************************************/
void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  uint8_t Loc_Status;

  
  char PayloadString[length + 1 ];
  /* convert payload in string */
  for(byte i=0;i<length;i++)
  {
    PayloadString[i] = payload[i];
  }
  PayloadString[length] = '\0';

  /* Debug */
  DEBUG_MQTT_RECEIVER

  StaticJsonBuffer<250> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(PayloadString);

  if (!root.success()) 
  {
    Serial.println("JSon parseObject() failed");
  } 
  else 
  {
    Serial.println("JSON message parsed succesfully");
  }

  /********************************************************************************************/
  /********************      HANDLING OF Received MQTT TOPICS WITH JASON     ******************/ 
  /********************************************************************************************/
  
  /*+++++++++++++++++++++++++++++ Set Strip's +++++++++++++++++++++++++++++++++++++++*/ 
  if(strcmp(topic, TOPIC_STRIP_1_PROGRAM_1_SET_ON)==0)
  {
    if(root.containsKey("status")) 
    {
      Loc_Status = root["status"];
      Serial.print("status strip 1 program 1 set:");
      Serial.println(Loc_Status, DEC);
      if(Loc_Status == 1)
      {
         StripProgram = STRIP_PROGRAM_01;
      }
      else
      {
         StripProgram = STRIP_PROGRAM_OFF;
      }
    }
  }
  if(strcmp(topic, TOPIC_STRIP_1_PROGRAM_2_SET_ON)==0)
  {
    if(root.containsKey("status")) 
    {
      Loc_Status = root["status"];
      Serial.print("status strip 1 program 2 set:");
      Serial.println(Loc_Status, DEC);
      if(Loc_Status == 1)
      {
        color_format.hue = root["hue"];
        color_format.sat = root["saturation"];
        color_format.val = root["brightness"];
        Serial.print("hue: ");
        Serial.print(color_format.hue);
        Serial.print("\t sat: ");
        Serial.print(color_format.sat);
        Serial.print("\t val: ");
        Serial.println(color_format.val);

        StripProgram = STRIP_PROGRAM_02;
      }
      else
      {
         StripProgram = STRIP_PROGRAM_OFF;
      }

    }
  }
  if(strcmp(topic, TOPIC_STRIP_1_PROGRAM_3_SET_ON)==0)
  {
    if(root.containsKey("status")) 
    {
      Loc_Status = root["status"];
      Serial.print("status strip 1 program 3 set:");
      Serial.println(Loc_Status, DEC);
      if(Loc_Status == 1)
      {
        // Get rid of '#' and convert it to integer
        const char* PtrColor = root.get<char*>("color"); // template version of get()
        long number = strtol( PtrColor, NULL, 16);
        //Serial.println(number);
        // Split them up into r, g, b values
        Color_Val_R = number >> 16;
        Color_Val_G = number >> 8 & 0xFF;
        Color_Val_B = number & 0xFF;
    
        Serial.print("red: ");
        Serial.print(Color_Val_R);
        Serial.print("\t green: ");
        Serial.print(Color_Val_G);
        Serial.print("\t blue: ");
        Serial.println(Color_Val_B);

        StripProgram = STRIP_PROGRAM_03;
      }
      else
      {
        StripProgram = STRIP_PROGRAM_OFF;
      }
    }
  }
}

/**************************************************************************************************
Function: mqttconnect()
Argument: void
return: void
**************************************************************************************************/
void mqttconnect(void)
{
  /* Loop until reconnected */
  while (!client.connected()) 
  {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "esp32-home-switch-strips-mqtt";
    /* connect now */
    if (client.connect(clientId.c_str(), serverUsername.c_str(), serverPassword.c_str()))
    {
      Serial.println("connected");
      /* subscribe topic's */
      client.subscribe(TOPIC_STRIP_1_PROGRAM_1_SET_ON);
      client.subscribe(TOPIC_STRIP_1_PROGRAM_2_SET_ON);
      client.subscribe(TOPIC_STRIP_1_PROGRAM_3_SET_ON);
      
    } 
    else 
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
      mqttRetryAttempt++;
      if (mqttRetryAttempt > 5) 
      {
        Serial.println("Restarting!");
        ESP.restart();
      }
    }
  }
}

/**************************************************************************************************
Function: Strip_Program_Cyclic()
Argument: void
return: void
**************************************************************************************************/
void Strip_Program_Cyclic(void)
{
	switch (StripProgram)
	{
		case STRIP_PROGRAM_OFF:
			Strip_Program_Off_Handle();
		break;
		case STRIP_PROGRAM_01:
			Strip_Program_01_Handle();
		break;
	    case STRIP_PROGRAM_02:
			Strip_Program_02_Handle();
		break;
    case STRIP_PROGRAM_03:
      Strip_Program_03_Handle();
    break;
		default:
		break;
		
	}
}

/**************************************************************************************************
Function: Strip_Program_Off_Handle()
Argument: void
return: void
**************************************************************************************************/
void Strip_Program_Off_Handle(void)
{
	for(int j = 0; j < NUM_LEDS; j++)
    {
       leds[j] = CHSV(0, 0,  0);
    }
    FastLED.show();	
}

/**************************************************************************************************
Function: Strip_Program_01_Handle()  (music detection with microphone)
Argument: void
return: void
**************************************************************************************************/
void Strip_Program_01_Handle(void)
{
    //volume
    unsigned long startMillis= millis();  // Start of sample window
    unsigned int peakToPeak = 0;   // peak-to-peak level
 
    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;
   
	//fft
	for(int i=0; i<SAMPLES; i++)
	{
		microseconds = micros();    //Overflows after around 70 minutes!

		vReal[i] = analogRead(0);
		vImag[i] = 0;

		while(micros() < (microseconds + sampling_period_us)){}
	}
	FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
	FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
	double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
	
	// collect data for 50 mS
	while (millis() - startMillis < sampleWindow)
	{
		sample = analogRead(DATA_MIC_INPUT);
		if (sample < 1024)  // toss out spurious readings
		{
			if (sample > signalMax)
			{
				signalMax = sample;  // save just the max levels
			}
			else if (sample < signalMin)
			{
				signalMin = sample;  // save just the min levels
			}
		}
	}
	peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
	double volts = (peakToPeak * 5.0) / 1024;  // convert to volts


	//smoothing
	// subtract the last reading:
	total = total - readings[readIndex];
	// read from the sensor:
	readings[readIndex] = peak;
	// add the reading to the total:
	total = total + readings[readIndex];
	// advance to the next position in the array:
	readIndex = readIndex + 1;

	// if we're at the end of the array...
	if (readIndex >= numReadings)
	{
		// ...wrap around to the beginning:
		readIndex = 0;
	}

	// calculate the average:
	average = total / numReadings;

  // Serial.println(average);
  // Serial.println(map(peakToPeak, 1, 400  , 0, 255));
  
  for(int j = 0; j < NUM_LEDS; j++)
  {
     leds[j] = CHSV(map(average, 200, 1500, 0, 150), 255,  map(peakToPeak, 1, 400, 0, 255));
  }
  FastLED.show();	
}

/**************************************************************************************************
Function: Strip_Program_02_Handle()   (Saturation, Brightmess, value)
Argument: void
return: void
**************************************************************************************************/
void Strip_Program_02_Handle(void)
{
    for(int j = 0; j < NUM_LEDS; j++)
    {
       leds[j] = CHSV( color_format.hue, color_format.sat, color_format.val); 
    }
    FastLED.show(); 
}

/**************************************************************************************************
Function: Strip_Program_03_Handle()  (color setting)
Argument: void
return: void
**************************************************************************************************/
void Strip_Program_03_Handle(void)
{
    for(int j = 0; j < NUM_LEDS; j++)
    {
       //leds[j] = CRGB( Color_Val_R, Color_Val_G, Color_Val_B);  //red and green are swapped ???
       leds[j] = CRGB( Color_Val_B, Color_Val_R, Color_Val_G); 
    }
    FastLED.show(); 
}
