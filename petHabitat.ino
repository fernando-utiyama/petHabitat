#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <Stepper.h>

#include "SinricProTemperaturesensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define APP_KEY           "xxx"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "xxx"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define SWITCH_ID         "xxx"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         9600                // Change baudrate to your need

#define TEMP_SENSOR_ID    "xxx"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define EVENT_WAIT_TIME   30000               // send event every 10 seconds

#define ONE_WIRE_BUS 17

const int stepsPerRevolution = 500; //change this to fit the number of steps per revolution
const int engineSpeed = 50;  // change this to fit the speed
// for your motor

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

bool deviceIsOn = true;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

// initialize the stepper library on pins 8 through 11:
//esp in1, in3, in2, in4
Stepper myStepper(stepsPerRevolution, 26, 14, 27, 12);

bool onPowerState(const String &deviceId, bool &state) {
  Serial.println(deviceId);
  Serial.println(state);
  if (deviceId == SWITCH_ID && state){
      myStepper.step(stepsPerRevolution);
      delay(5);
      myStepper.step(stepsPerRevolution);
      Serial.println("Alimentando");
      return true;
  }
  if (deviceId == TEMP_SENSOR_ID && state){
      Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state?"on":"off");
      deviceIsOn = state; // turn on / off temperature sensor
      Serial.println("Sensor");
      return true;
  }
  return true;                                // request handled properly
}


void setup() {
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.

    // put your setup code here, to run once:
    Serial.begin(115200);
    
    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    //wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("Pet Habitat"); // anonymous ap
    res = wm.autoConnect("Pet Habitat","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected");
    }

    // Start up the library
    sensors.begin();
    myStepper.setSpeed(engineSpeed);
    setupSinricPro();
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];   // create new switch device
  mySwitch.onPowerState(onPowerState);                // apply onPowerState callback
  SinricPro.begin(APP_KEY, APP_SECRET);               // start SinricPro
  Serial.println("Alimentador criado");

  
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  mySensor.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);  
  Serial.println("Temperatura criado");
}

void loop() {
   SinricPro.handle();                         // handle SinricPro commands
   handleTemperaturesensor();
}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too much events
 * - Get actual temperature and humidity and check if these values are valid
 * - Compares actual temperature and humidity to last known temperature and humidity
 * - Send event to SinricPro Server if temperature or humidity changed
 */
void handleTemperaturesensor() {
  if (deviceIsOn == false) {
    Serial.println("Sensor desligado");
    return; // device is off...do nothing
  }

  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  float temperature = sensors.getTempCByIndex(0);

  if (isnan(temperature)) { // reading failed... 
    Serial.println("Error: Could not read temperature data");
    return;                                    // try again next time
  } 

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event

  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Enviou temperatura: %2.1f Celsius", temperature);
  } else {  // if sending event failed, print error message
    Serial.printf("temperatura: %2.1f Celsius", temperature, "/n");
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}
