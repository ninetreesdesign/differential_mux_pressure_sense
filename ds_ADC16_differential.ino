/*
try reading load cell (pressure sensor) directly from internal ADCs (12-bit)

Modified to use external 4:1 differential 16-bit ADC
By selecting other ADC input, read the mux-selected channel through x100 instr amplifier

Can see with 12-bits for larger forces
Better with 16-bits
With amp, can see two grams on the cantilever of 6mV Amazon load cell

5/19 this is broken... see differential_16
 */

//#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <SPI.h>
#include <Wire.h>
#endif
#include <Adafruit_ADS1015.h>       // quad 16bit ADC (also used for the 12bit ADC)

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 20
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, 19, 18, 20); // u8g2 drive for Adafruit OLED

//int i2c_address_ads1115 = 0x48;  // base address when ADDR is connected to Gnd

String buf = " ";
const byte STR_MAX = 22;
char str[STR_MAX]       = "\0";
char str_prev[STR_MAX] = "\0";
// ROWS, COLUMNS
const int ROWS = 2;
char a[ROWS][22] = {
    "alpha  ",
    "beta   "
};
int led_pin = LED_BUILTIN;              // select the pin for the LED
// use these two pins as excitation voltage; option to modulate
int ex_pos = 16;
int ex_neg = 15;
int MUX_SEL_0 = 2;
int MUX_SEL_1 = 3;

int lineheight = 12;
byte WHT=1;
byte BLK=0;


const int ADC_BITS = pow(2,16);
const float adc_gain  = 1.0; //16.0;
const uint32_t sample_interval = 8;    // ms
const byte i2c_address_ads1115 = 0x48;  // base address when ADDR is cnnected to Gnd

Adafruit_ADS1115 ads1115;               // Use this for the 16-bit version of ADC ('1115) although lib name references the 12-bit device ('1015) */

byte mux_channel = 0;
int adc_channel = 0;
int num_readings = 8;
int print_nth_time = 10;

static int32_t adc_value;
float multiplier = 0.125F;  // mV
float amplified_sensor_voltage;
float v_diff;
//float v[20];        // store up to 20 values before printing
uint32_t t0;            // start value for ms timer
uint32_t measurement_count = 0;

void setup(void) {
    pinMode(led_pin, OUTPUT);
    pinMode(ex_pos, OUTPUT);
    pinMode(ex_neg, OUTPUT);
    pinMode(MUX_SEL_0, OUTPUT);
    pinMode(MUX_SEL_1, OUTPUT);
    // flash to indicate program starting
    for (int i = 0; i<3; i++) {
        digitalWrite(led_pin,1);
        delay(20);
        digitalWrite(led_pin,0);
        delay(80);
    }

    analogReadResolution(16);   // set resolution for the internal ADC (will shift left to match HW)

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    // start with clr to avoid splash scrn
    display.clearDisplay();   // clears the screen and buffer
    display.display(); // show splashscreen

    // higher range assumes a 5V supply
    // 0x0000   ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit =   0.1875mV (default)
    // 0x0200   ads1115.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit =   0.125mV
    // 0x0400   ads1115.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit =   0.0625mV
    // 0x0600   ads1115.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit =   0.03125mV
    // 0x0800   ads1115.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit =   0.015625mV
    // 0x0a00   ads1115.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit =   0.0078125mV = 0.125/16
    if (adc_gain == 1.0) {
        ads1115.setGain(GAIN_ONE);
    }
    else if (adc_gain == 16.0) {
        ads1115.setGain(GAIN_SIXTEEN);
    }
    multiplier /= adc_gain;  // mV  make size of input voltage amount smaller by this value

    // ads1115.begin(i2c_address_ads1115);    // no argument is default I2C address of 0 offset from base 0x48
    ads1115.begin();

    int val = 0x02E3;  // 0 000, 001 0, 111 0, 0 0 11 init to defaults except fastest sampling rate
    Wire.beginTransmission(i2c_address_ads1115);
    Wire.write(byte(01));        // sends instruction byte
    Wire.write(val);             // sends data byte
    Wire.endTransmission();      // stop transmitting

    Serial.begin(57600); // rate doesn't matter for teensy routed to USB console
    while (!Serial && (millis() < 5000)) {
    } // include timeout if print console isn't opened
    Serial.println("USB Serial Print initialized*");
    Serial.println("Differential reading");

// use two io pins to set bias V; Allows for modulation later...
    digitalWrite(ex_pos,1);     // 3.3V
    digitalWrite(ex_neg,0);     // 0V

    //i2cScan();

    t0 = millis();

    v_diff = getV(num_readings, 0, 0);    // mV
    Serial.print(v_diff);
    Serial.print(" ");
    v_diff = getV(num_readings, 0, 1);    // mV
    Serial.print(v_diff);
    Serial.print(" ");
    v_diff = getV(num_readings, 0, 2);    // mV
    Serial.print(v_diff);
    Serial.print(" ");
    v_diff = getV(num_readings, 0, 3);    // mV
    Serial.print(v_diff);
    Serial.print(" \n\n");

    v_diff = ads1115.readADC_Differential_0_1(); // read dif inputs 0 & 1
    Serial.print(v_diff);
    Serial.print(" \n\n");
    v_diff = ads1115.readADC_Differential_2_3(); // read dif inputs 2 & 3
    Serial.print(v_diff);
    Serial.print(" \n\n");
    delay(500);
}


void loop(void) {                           // ----------------------------------------------------------------
    static int i = 0;
    static uint32_t t;
    static int count = 0;
    float v_print_avg = 0;

    t = millis();
    i++;
    digitalWriteFast(led_pin,1);            // indicate sample rate on board LED
    delay(sample_interval/2);
    digitalWriteFast(led_pin,0);
    delay(sample_interval/2);

    // copyString(str_prev, str);   // dest, src
    num_readings = 16;
    adc_channel = 0;
    mux_channel = 0;
    v_diff = getV(num_readings, adc_channel, mux_channel);    // mV
    dtostrf(v_diff, 8, 3, str);
//   measurement_count++;
    Serial.print("+ ");
    P();
    delay(1);
    adc_channel = 0;
    mux_channel = 3;
    v_diff = getV(num_readings, adc_channel, mux_channel);    // mV
    dtostrf(v_diff, 8, 3, str);
//   measurement_count++;
    Serial.print("^ ");
    P();

//    adc_channel = 2;
//    mux_channel = 0;
//    v_diff = getV(num_readings, adc_channel, mux_channel);    // mV
//    dtostrf(v_diff, 8, 3, str);
//    measurement_count++;
//    P();
//
//    mux_channel = 3;
//    v_diff = getV(num_readings, adc_channel, mux_channel);    // mV
//    dtostrf(v_diff, 8, 3, str);
//    measurement_count++;
//    P();

}   // end loop


float getV(int num_readings, int adc_ch, int mux_ch) {
    int32_t adc_n;
    selectMuxChannel(mux_ch);
    adc_n = getADC(num_readings, adc_ch);
    Serial.println(adc_n);
    adc_value = (float)adc_n/num_readings; // copy to global
    amplified_sensor_voltage = adc_value * multiplier;     // mV
    return amplified_sensor_voltage;
}


int32_t getADC(int num_readings, int adc_channel) {
    // returns value * num readings; rounding external to fctn
    // read ADS1115 16-bit ADC with differential mode on 1st channel pair
    // num_readings: number of times to read; output avg
    // this takes about XXX1.59ms for n=64, XXXms for n=256
    int32_t adc_offset = 0.000;
    int32_t adc_n = 0;
    // error check, constrain
    if (num_readings < 1)
        num_readings = 1;
    // ignore first reading
    for (int i = 0; i < num_readings+1; i++) {
        if (adc_channel == 0 && i > 0)  adc_n += ads1115.readADC_Differential_0_1(); // read dif inputs 0 & 1
        if (adc_channel == 2 && i > 0)  adc_n += ads1115.readADC_Differential_2_3(); // read dif inputs 2 & 3
        if (adc_channel == 1 && i > 0) Serial.println("error: no ch 1");
        if (adc_channel == 3 && i > 0) Serial.println("error: no ch 3");
    }
    //adc_n /= num_readings;
    //adc_n -= adc_offset;   // observed value

//    // temp
//    Serial.print(" adc counts: ");
//    Serial.print(adc_n);

    return adc_n;
}



void selectMuxChannel(byte n) {
// use selection to select two mux bits A,B
    byte A = (n & 0x01) > 0;
    byte B = (n & 0x02) > 0;
    digitalWrite(MUX_SEL_1,B);
    digitalWrite(MUX_SEL_0,A);
    //temp
//    Serial.print(B);   // 1 bit
//    Serial.print(A);   // 0 bit
//    Serial.print(" ");
//    Serial.println(n);
}


//float getV(int n) {
//// read two ADC channels of Teensy 3.x and finds difference
//// this takes about 1.59ms for n=64, 6.35ms for n=256
//    int32_t adc_n = 0;
//    int32_t k2 = 0;
//    for (int i = 0; i < n; i++) {
//        adc_n += analogRead(A0);
//        k2 += analogRead(A1);
//    }
//    adc_n /= n;
//    k2 /= n;
//    return (float)(k2 - adc_n)*3.3/ADC_BITS;
//}


// print stuff in str
void P() {
    static int count = 0;
    int print_nth_time = 10;
    static float v_print_avg = 0;
    static float v[20];

// to OLED
    // use clr; fast, no flicker
    const byte h = 8;
    const byte w = 6;
    int fsize;
    display.setTextColor(1);        // 1=WHITE, 0=BLACK

    display.clearDisplay();         // clears the screen and buffer
    fsize=1;
    display.setTextSize(fsize);     // font size [1|2|3]
    display.setCursor((18-1)*w*fsize,(4-1)*h*fsize);
    display.print("mV");            // print string at xy location

    fsize=2;
    display.setTextSize(fsize);     // font size [1|2|3]
    display.setCursor((1-1)*w*fsize,(2-1)*h*fsize);
    display.print(str);             // print string at xy location
    display.display();              // refresh the screen


// to console terminal
    Serial.println(str);
    dtostrf(-99, 8, 0, str);

    return;
///   Serial.print(i);
//    Serial.print(" \t");    Serial.print(str);    Serial.print(" mV \n");

// to get throughput without serial port limitation,
// measure n and store; then print all at once
    v[count] = v_diff; // V

//       // print...
    count++;
    if (count > print_nth_time-1) {
        count = 0;
        for (int i = 0; i < print_nth_time; i++) {
            buf += v[i];
            buf += " ";
            v_print_avg += v[i];
        }
        buf += "\n";
        // Serial.print(String(millis()-t0) + "  " + String(measurement_count) + "  ");
        //   Serial.print(  String(measurement_count) + "  ");
        Serial.print(" a=" + String(adc_channel) + "  m=" + String(mux_channel));
        Serial.print(str);
        Serial.print("\n");
        // Serial.print(String(print_nth_time) + " mV  " + buf);
        //Serial.print(String(v_print_avg/print_nth_time) + " mV  " + buf);
        buf = " ";
        v_print_avg = 0;

    }
}


void printOLED(char sbuf[], byte xpos, byte ypos, byte fsize, byte clr) {
// (chars and lines are defined to start at 1,1 for upper-left)
    const byte h = 8;
    const byte w = 6;
    if (xpos < 1) xpos = 1;
    if (ypos < 1) ypos = 1;
    display.clearDisplay();   // clears the screen and buffer
    display.setTextColor(clr);      // 1=WHITE, 0=BLACK
    display.setTextSize(fsize);     // font size [1|2|3]
    display.setCursor((xpos-1)*w*fsize,(ypos-1)*h*fsize);
    display.print(sbuf);            // print string at xy location
    display.display();              // refresh the screen
}


void copyString(char s2[], char s1[]) {
    byte str_size = STR_MAX-1;  //sizeof(s1);
    strncpy(s2, s1, str_size);  // dest, src, size
    str_prev[str_size - 1] = '\0';
}


void i2cScan() {
    Serial.println ("\nI2C scanner. Scanning ...");
    static int count = 0;

    Wire.begin();
    for (int i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            Serial.print ("Found address: ");
            Serial.print (i, DEC);
            Serial.print (" [0x" + String(i, HEX) + "]\n");
            count++;
            delay (1);
        }
    }
    Serial.print   ("*** Total of ");
    Serial.print   (count, DEC);
    Serial.println (" devices.\n");
}




