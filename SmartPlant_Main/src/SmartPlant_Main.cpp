/* 
 * Project Smart Houseplant Watering System
 * Author: Rudy Parra
 * Date: Nov 7th , 2023
 * Description: This code provides the capability to monitor the mositure levels
 *              of the soil and its surrounding evironment like dust, temperature, 
 *              and air quality to determine when is a good time the plant should
 *              be watered with a motorized water pump. 
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "math.h"
#include "Air_Quality_Sensor.h"
#include "Arduino.h"
#include "HX711.h"
#include <Adafruit_BME280.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "credentials.h"

/********* Global State and Variables ***********/
TCPClient TheClient;
String DateTime, TimeOnly, DateOnly;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

/********* Feeds ********/
//Setting up the Feeds to publush or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/<freedname>
Adafruit_MQTT_Publish aqNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airquality");
Adafruit_MQTT_Publish dqNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustquality");
Adafruit_MQTT_Publish msNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisturenumbers");
Adafruit_MQTT_Publish tsNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturenumbers");
Adafruit_MQTT_Publish psNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressurenumbers");
Adafruit_MQTT_Publish huNumbers = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humiditynumbers");
Adafruit_MQTT_Subscribe buttonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff"); 



//Setting AirQuality Objects
AirQualitySensor airqualitysensor(A5);
int current_quality = -1;

//Setting OLED Display Objects
Adafruit_SSD1306 display(-1);

//Setting up Temperature Sensor Object and integers
Adafruit_BME280 bme;
int hexAddress = 0x76; // BME280 I2C address (you can change this to 0x77 if necessary)
int status;

//Setting Dust Sensor Integers and Unsigned Values
int dustSensor = A2;
unsigned long duration;
unsigned long startTime;
unsigned long sampleTime = 30000; //Sample 30 Seconds
unsigned long lowPulseOccupancy = 0;
float ratio = 0;
float concentration = 0;

//Setting Moisture Sensor Integers and Variables
int moistSensor = A1;
int moistValue;
float subValue;
unsigned long waterTime = 60000; //Sample 30 Seconds

//***** Setting Water Pump Integers *****/
int waterPump = D16;

//Setting i as an int
int i;

/******** Declaring Functions ********/
void Time_Keeper ();
void Dust_Sensor();
void Air_Quality_Sensor ();
void Moisture_Sensor ();
void Temperature_Sesnor ();
void OLED_Display_Information (float tempF, float pressInHg, float humidRH);
void MQTT_connect();
bool MQTT_ping();

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

    void setup() {

        //Setting Serial Monitor
        Serial.begin(9600);

        //Connecting to the Internet but not the Particle Cloud
        WiFi.on();
        WiFi.connect();
        while(WiFi.connecting()){
        Serial.printf(".");
        }
        Serial.printf("\n\n");

        //Setting Time Millis
        startTime = millis(); //get the current time;

        //Setting OLED DISPLAY Parameters
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.display();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.display();

        //Setting Up TIme Parameters
        Time.zone(-7);        //MST = -7, MDT = -6
        Particle.syncTime();  //Sync time with Particle Cloud

        //Setting DustSensor Value
        pinMode(dustSensor, INPUT);

        //Setting Water Pump
        pinMode(waterPump, OUTPUT);

        //Setting Capacitive Soil Moisture Sensor
        pinMode(moistSensor, INPUT);

        // Initialize BME280
        status = bme.begin(hexAddress);
        if (!status) {
            Serial.println("BME280 failed to start");
        }

        // Setup MQTT subscription
        mqtt.subscribe(&buttonFeed);

        
    }

    void loop() {
        //Calling Function Values
        Time_Keeper();
        Dust_Sensor();
        Air_Quality_Sensor ();
        Moisture_Sensor ();
        Temperature_Sesnor ();
        MQTT_connect();
        MQTT_ping();
    
    }


    //***** Functions *****/

    void Time_Keeper () {
        //Implementing Time in OLED

        DateTime = Time.timeStr();    //Current Date and Time from Particle Time Class
        TimeOnly = DateTime.substring(11,19); //Extract the time from the DateTime String
        DateOnly = DateTime.substring(0,10); //Extracts the date from the DateTIme
            //delay(10000);
        //%s prints an array of char
        //the .c_str() method converts a String to an array of char
        
    }

    void Dust_Sensor () {
         //***** Dust Sensor Portion *****/
        startTime = millis();
        duration = pulseIn(dustSensor, LOW);
        lowPulseOccupancy = lowPulseOccupancy+duration;
        
        if ((millis()-startTime)> sampleTime) // if the sample time == 30s
        {
        ratio = lowPulseOccupancy/(sampleTime*10.0); // Integer [ercentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; //using spec sheet curve
        Serial.printf("Ratio = %f\n", ratio);
        Serial.printf("Concentration = %f\n", concentration);
        lowPulseOccupancy = 0;

        
        if ((millis()-startTime)> sampleTime) // if the sample time == 30s
        {
        //******** ADAFRUIT CODE ********
        dqNumbers.publish(ratio);
        }
        }
    }

    void Air_Quality_Sensor () {
        //***** Air Quality Sensor Portion *****/
        current_quality = airqualitysensor.slope();
        
        if (current_quality >= 0) // if a valid data returned.
            {
            if (current_quality ==0)
            Serial.println("High pollution! Force signal active\n");
            else if (current_quality == 1)
                Serial.println("High Pollution!\n");
            else if (current_quality == 2)
                Serial.println("Low pollution!\n");
            else if (current_quality == 3)
                Serial.println("Fresh air\n");

             if ((millis()-startTime)> sampleTime) // if the sample time == 30s
            {
            //******** ADAFRUIT CODE ********
            aqNumbers.publish(current_quality);
            }
            }
    }

    void Moisture_Sensor () {
        //***** Moisture Sensor Portion *****/
        //Obtaining Moisture
        moistValue = analogRead(moistSensor);
        Serial.printf("Moisture Level = %i\n", moistValue);
        //***** ADAFRUIT PUBLISH *****/
        msNumbers.publish(moistValue);

        if ((millis()-startTime)> waterTime) // if the sample time == 30s
        {
        if (moistValue >= 3350){
            digitalWrite(waterPump ,HIGH);
            delay(500);
            digitalWrite(waterPump,LOW);
        }
        }

        // this is our 'wait for incoming subscription packets' busy subloop 
        Adafruit_MQTT_Subscribe *subscription;
        while ((subscription = mqtt.readSubscription(100))) {
            if (subscription == &buttonFeed) {
            subValue = atof((char *)buttonFeed.lastread);
            
            digitalWrite(waterPump ,HIGH);
            delay(500);
            digitalWrite(waterPump,LOW);
            }
        }
    }

    void Temperature_Sesnor () {
        //***** Temperatrue Sensor Portion *****/
        float tempC = bme.readTemperature();
        float pressPA = bme.readPressure();
        float humidRH = bme.readHumidity();
        float tempF = (tempC * 9.0 / 5.0) + 32;  // Convert temperature to Fahrenheit
        float pressInHg = pressPA / 3386.39; // Convert pressure to inHg
        // Display data on Serial Monitor
        Serial.printf("Temperature: %.2f °F\n", tempF);
        Serial.printf("Pressure: %.2f inHg\n", pressInHg);
        Serial.printf("Humidity: %.2f %%\n", humidRH);

    
        //***** ADAFRUIT PUBLISH *****/
        tsNumbers.publish(tempF);
        psNumbers.publish(pressInHg);
        huNumbers.publish(humidRH);
    

        OLED_Display_Information (tempF, pressInHg, humidRH);

    }

    void OLED_Display_Information (float tempF, float pressInHg, float humidRH) {
        //***** Displaying Information on OLED Screen *****/
        static unsigned int screenFlipTime;
        static int currentOLED;
            if((millis()- screenFlipTime ) > 2000) {
                currentOLED++;

                if(currentOLED>2){
                currentOLED = 0;
                }
                switch(currentOLED){
                    case 0:
                    display.clearDisplay();
                    display.setTextSize(2);
                    display.setTextColor(WHITE);
                    display.setCursor(0,5);
                    display.printf("Temp: \n  %0.1f%c",tempF);
                    display.setCursor(0,40);
                    display.display();
                    break;
                    case 1:
                    display.clearDisplay();
                    display.setTextSize(2);
                    display.setTextColor(WHITE);
                    display.setCursor(0,5);
                    display.printf("Pressure: \n %0.1f", pressInHg);
                    display.setCursor(0,40);
                    display.display();
                    break;
                    case 2:
                    display.clearDisplay();
                    display.setTextSize(2);
                    display.setTextColor(WHITE);
                    display.setCursor(0,5);
                    display.printf("Humidity: \n %0.1f%c",humidRH);
                    display.setCursor(0,40);
                    display.display();
                    break;
                    }
                screenFlipTime = millis();
                }
    }

    void MQTT_connect() {
        int8_t ret;

        //Return if already connected.
        if(mqtt.connected()) {
        return;
        }

        Serial.print("Connecting to MQTT... ");

        while ((ret = mqtt.connect()) != 0) { // Connection will return 0 for connected
            Serial.printf("Error code %s\n",mqtt.connectErrorString(ret));
            Serial.printf("Retrying MQTT connection in 5 seconds.....\n");
            mqtt.disconnect();
            delay(5000); // Waits 5 seconds and tries again
        }
        Serial.printf("MQTT Connected!\n");
    }

        bool MQTT_ping() {
        static unsigned int last;
        bool pingStatus;

        if ((millis()-last)>120000) {
            Serial.printf("Pinging MQTT \n");
            pingStatus = mqtt.ping();
            if(!pingStatus) {
                Serial.printf("Disconnecting \n");
                mqtt.disconnect();
            }
            last = millis();
        }
        return pingStatus;
        }

