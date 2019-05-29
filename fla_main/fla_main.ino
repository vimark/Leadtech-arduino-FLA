#include <Event.h>
#include <Timer.h>

//using Timer libary from https://playground.arduino.cc/Code/Timer/

//based on Gizduino X schematic
//pin 14 is PD7
//pin 15 is PD6
//pin 16 is PD5
//pin 17 is PD4
//pin 12 is PB1 
//pin 13 is PB3

//pin order
//port D -  4| 5| 6| 7 (high nibble)
//pin    - 17|16|15|14
//sensor -  1| 2| 3| R

#define pulse_length_seconds 1

//input pin assignment
int in_sensor1 = 17;
int in_sensor2 = 16;
int in_sensor3 = 15;
int in_reset = 14;
int out_flasher = 13;       // the number of the output pin
int out_siren = 12;

int state = LOW;      // the current state of the output pin
byte sensors;           // the current reading from the input pin
int reset; //reset input data
byte previous = B00000000;    // the previous reading from the input pin
int sensor1, sensor2, sensor3; //sensor values storage
int timer_id;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

Timer t; //timer instance

void setup()
{
  DDRD = DDRD & B00001111; //set pin 14, 15, 16 and 17 as input
  pinMode(out_flasher, OUTPUT);
  pinMode(out_siren, OUTPUT);
}

void loop()
{
  sensors = PIND & B01110000; //read all sensor inputs
  reset = digitalRead(in_reset);

 if(sensors !=  previous && reset == LOW && millis() - time > debounce){ //if the sensors are triggered, reset button not pressed and the debounce timer expires
  switch(sensors){
  case B00010000: //sensor 1 activated
    digitalWrite(out_flasher, HIGH);
    t.stop(timer_id); //make sure to stop siren when sensor 2 stops triggering e.g. after sensor 2 was untriggered or water level going low
    digitalWrite(out_siren, LOW); //sometimes the toggle transistion stops at HIGH, force to LOW
    break;
  case B00110000: //sensor 1 & 2 activated
    digitalWrite(out_flasher, HIGH);
    timer_id = t.oscillate(out_siren, pulse_length_seconds * 1000, HIGH); // siren toggle
    break;
  case B01110000: //sensor 1, 2 & 3 activated
    digitalWrite(out_flasher, HIGH);
    t.stop(timer_id); //stop siren oscillation
    digitalWrite(out_siren, HIGH);
    break;
  default: //otherwise all output are turned off, must send an error msg?
    digitalWrite(out_flasher, LOW);
    digitalWrite(out_siren, LOW);
    t.stop(timer_id);
    break;
 }
 time = millis(); //store time
 }
 else if(reset == HIGH && millis() - time > debounce){ //cancel all alarms
  digitalWrite(out_flasher, LOW);
  digitalWrite(out_siren, LOW);
  t.stop(timer_id);

  time = millis(); //store time
 }
 previous = sensors; //store current sensor values
 t.update(); //update timer
}
