// This #include statement was automatically added by the Particle IDE.
#include "pid/pid.h"

// This #include statement was automatically added by the Particle IDE.
#include "blynk/blynk.h"

// This #include statement was automatically added by the Particle IDE.
#include "DS18B20/DS18B20.h"

// This #include statement was automatically added by the Particle IDE.
#include "blynk/blynk.h"

// This #include statement was automatically added by the Particle IDE.
#include "pid/pid.h"

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "put in your own blynk auth";

//pins
int tempSensorPin = D2; //DS18B20 Temperature sensor pin
int heatElementRelayPin = D3;

DS18B20 ds18b20 = DS18B20(tempSensorPin); 
unsigned int publishInterval = 5000;
unsigned int nextPublishTime;
int DS18B20nextSampleTime;
int DS18B20_SAMPLE_INTERVAL = 1000;
int dsAttempts = 0;
double temperature;
double targetTemperature;
double pidAnalogOutput;

bool heatElementOn = false;
bool on = false;

int windowSize = 10000;
unsigned long windowStartTime;

//PID tuning Parameters
double kp=1000, ki=0.5, kd=0.1;
PID myPID(&temperature, &pidAnalogOutput, &targetTemperature, kp, ki, kd, PID::DIRECT);

void setup() {
    targetTemperature = 30;
    
    Particle.syncTime();
    pinMode(tempSensorPin, INPUT);
    pinMode(heatElementRelayPin, OUTPUT);
    digitalWrite(heatElementRelayPin,HIGH); 
    
    Blynk.begin(auth);
    
    //PID
    windowStartTime = millis();
    myPID.SetOutputLimits(0, windowSize);
    myPID.SetMode(PID::AUTOMATIC);
}

void loop() {
    Blynk.run();
    unsigned long now = millis();
  
    if (now > DS18B20nextSampleTime) {
        getTemp();
    }

    if (on) {
        
        myPID.Compute();
        
        unsigned long windowTime = now - windowStartTime;
        if (windowTime > windowSize) { //time to shift the Relay Window
            windowStartTime += windowSize;
        }
        if (pidAnalogOutput > windowTime) { 
            heatElementOn = true;
            digitalWrite(heatElementRelayPin,LOW); 
        } else {
            heatElementOn = false;
            digitalWrite(heatElementRelayPin,HIGH); 
        }
 
    } else {
        heatElementOn = false;
        digitalWrite(heatElementRelayPin,HIGH); 
    }
    if (now > nextPublishTime) {
        publishData();
    }
}

void publishData(){
    
    char publishString[60];
    double onRate = 0;
    if (on) {
        onRate = (pidAnalogOutput / windowSize) * 100;
    }
    sprintf(publishString, "temperature: %0.2f°C, on-rate: %0.2f%%, target: %0.2f°C", temperature, onRate, targetTemperature);
    Particle.publish("Status",publishString);
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V4, onRate);
  
    nextPublishTime = millis() + publishInterval;
}

void getTemp(){
    double oldTemperature = temperature;
    if(!ds18b20.search()){
      ds18b20.resetsearch();
      temperature = ds18b20.getTemperature();
      while (!ds18b20.crcCheck() && dsAttempts < 4){
        dsAttempts++;
        Particle.publish("Temperature reading error", "Retry attempt nr: " + String(dsAttempts));
        if (dsAttempts == 3){
          delay(1000);
        }
        ds18b20.resetsearch();
        temperature = ds18b20.getTemperature();
        continue;
      }
      dsAttempts = 0;
      
      DS18B20nextSampleTime = millis() + DS18B20_SAMPLE_INTERVAL;
    }
    //False measurement
    if (temperature < 0) {
        temperature = oldTemperature;
    }
}

//Button Widget is writing to pin V3
BLYNK_WRITE(V3) { 
  on = (param.asInt() == 1); 
}

//Slider Widget is writing to pin V2
BLYNK_WRITE(V2) { 
  targetTemperature = param.asInt(); 
}



