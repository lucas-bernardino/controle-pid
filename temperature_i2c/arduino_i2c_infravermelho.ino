#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; // Default Address 0x48
float DCV = 0.0;

void setup(void)
{
Serial.begin(9600);
ads.begin();
}

void loop(void)
{
int16_t adc0;

adc0 = ads.readADC_SingleEnded(0);
DCV = (adc0 * 0.1875)/1000.0;
float tempIR = 252.01*DCV - 337.8;


//Serial.print("* AIN0 = "); Serial.print(adc0);
Serial.println(tempIR, 2);

delay(100);
}