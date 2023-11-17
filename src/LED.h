#include <Adafruit_NeoPixel.h>

// defining led and buzzer
#define PIN 2
Adafruit_NeoPixel led = Adafruit_NeoPixel(1, PIN);

int buzzer = 15; // buzzer pin is 15
unsigned long lastPeriodStart; // just defining the variables for the buzzer to beep without delay
const int onDuration = 500;
const int periodDuration = 1000;


// led blinking sequences
void led_init(){
    led.begin();
    led.show();
    led.setBrightness(100);
    pinMode(buzzer, OUTPUT);
    
    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 161, blink * 216, blink * 139); // white
    led.show();

    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 1567, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}

void led_pad_idle()
{
    led.begin();
    led.show();
    led.setBrightness(100);
    pinMode(buzzer, OUTPUT);
    
    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 0, blink * 255, blink * 0); // green
    led.show();
  
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 2451, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}

void led_pow_flight(){
    led.begin();
    led.show();
    led.setBrightness(75);
    pinMode(buzzer, OUTPUT);

    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 255, blink * 255, blink * 255); // white
    led.show();
    
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 1750, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}

void led_descent(){
    led.begin();
    led.show();
    led.setBrightness(75);
    pinMode(buzzer, OUTPUT);

    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 0, blink * 0, blink * 255); // blue
    led.show();
    
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 1234, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}

void led_landed(){
    led.begin();
    led.show();
    led.setBrightness(75);
    pinMode(buzzer, OUTPUT);

    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 255, blink * 255, blink * 0); // yellow
    led.show();
    
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 357, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}


void ledRed(){
    led.begin();
    led.show();
    led.setBrightness(75);
    pinMode(buzzer, OUTPUT);

    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 255, blink * 0, blink * 0); // yellow
    led.show();
    
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 1750, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}

void led_party(){
    led.begin();
    led.show();
    led.setBrightness(100);
    pinMode(buzzer, OUTPUT);

    int blink = (millis() % 1000) < 500; // 1000 is the period between blinks, 500 is the length of blink
    led.setPixelColor( 0, blink * 252, blink * 15, blink * 192);
    led.show();
    
    if (millis()- lastPeriodStart >= periodDuration){
        lastPeriodStart = millis();
        tone(buzzer, 2345, onDuration); // play 2451 Hz tone in background for 'onDuration'
    }
}