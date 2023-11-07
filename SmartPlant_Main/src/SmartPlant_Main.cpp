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
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_SSD1306.h"
#include "credentials.h"

/********* Global State and Variables ***********/
TCPClient TheClient;
String DateTime, TimeOnly;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

/********* Feeds ********/
//Setting up the Feeds to publush or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/