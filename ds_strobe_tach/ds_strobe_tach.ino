/*  Use Teensy to make a strobe tachometer
 *  Use adjustable input (dual pot) to set a frequency in RPM
 *    display frequency
 *    flash several LEDs in short pulse for strobe
 *
 *  See PWM and resolution at https://www.pjrc.com/teensy/td_pulse.html
 *  To generate pulse, use low-freq PWM method (couldn't get to work; changed to calculated software pulse
 *  be sure to use PWM-able pins (3 4 5 6 9... on T3.2)
 *
 *
*/

#define HWSERIAL Serial1
#define CONSOLE  Serial                 // USB port debug console (IDE)
#define E        echoString             // my little print shortcut

#include <U8g2lib.h>
#if 0
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#endif
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);
const bool INVERT_FLAG = 0;
const bool ON  = 1 ^ INVERT_FLAG;
const bool OFF = 0 ^ INVERT_FLAG;

/////////////////////////////////////////////////////////////////////////////////////
const String VERSION_NUM = "1.1.0";     // use usec for period
const bool PRINT_FLAG = 1;              // enable print of streaming data (in echoString)
const bool HWSERIAL_FLAG = 0;           // send msgs to hardware serial port
const bool CONSOLE_FLAG = 1;            // send msgs to standard USB output port
const float MAX_RPM = 1500.0;           // sets range of adjustment
const float MIN_RPM =   30.0;
/////////////////////////////////////////////////////////////////////////////////////

byte CLOCK_PIN_A = 5;
byte CLOCK_PIN_B = 6;
byte LED_PIN = 13;
byte ch_coarse = A7;
byte ch_fine = A8;
const float MAX_VAL_ANALOG = 65535.0;
char msg[80];                           // used for sprintf() container
float freq = 1.00;
float freq_rpm = freq * 60;

uint32_t period_us = 1000;
uint32_t period_us_prev = 0;
uint32_t period_scale = 1e6;

float duty_cycle = 0.05;                // fractional
uint32_t t_on;
uint32_t t_off;
uint32_t t0;
bool clock_flag = 0;

// intervals for various timed events
const uint16_t SENSORS_INTERVAL =  100;     // us
const uint16_t PRINT_INTERVAL   =  300;     // ms
const uint16_t LED_INTERVAL     = 1000;     // ms
// timer variables
elapsedMicros since_pulse = 0;              // us

elapsedMillis since_sensors = 0;            // ms
elapsedMillis since_print = 0;              // ms
elapsedMillis since_led = 0;                // ms

///
void setup() {
    pinMode(CLOCK_PIN_A, OUTPUT);
    pinMode(CLOCK_PIN_B, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    // LED startup splash
    for (int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH);        // turn the LED on (HIGH is the voltage level)
        digitalWrite(CLOCK_PIN_A, HIGH);    // turn the LED on (HIGH is the voltage level)
        delay(20);                          // wait for a time
        digitalWrite(LED_PIN, LOW);         // turn the LED off by making the voltage LOW
        digitalWrite(CLOCK_PIN_A, LOW);     // turn the LED off by making the voltage LOW
        delay(200);                         // wait for a time
    }
    digitalWrite(LED_PIN,1);
    //analogWriteFrequency(CLOCK_PIN_A, 50000);    // set PWM freq
    analogReadResolution(16);
    //analogWriteResolution(8);  

    // print header message
    E("Strobe Tachometer \n");
    E("Version " + VERSION_NUM + "\n");
    float x = -3.4567;
    float y = roundToNumDigits(x,2);
    float z = roundToNumDigits(x*x, 1);
      z = roundToNumDigits(x*x, 0);
      
    oledInit(); 
    t0 = millis();
}


// --- main loop --------------------------------------------------------------------------------
void loop() {

    // generate the strobe pulse
    if (since_pulse >= period_us) {             // start the pulse
        since_pulse = 0;
        if (clock_flag == 0) {
            digitalWriteFast(CLOCK_PIN_A,1);
            digitalWriteFast(CLOCK_PIN_B,1);    // extra pin for additional lights
            clock_flag = 1;
        }
    }

    if (since_pulse >= t_on) {                  // pulse goes low for remainder of period
        if (clock_flag == 1) {
            digitalWriteFast(CLOCK_PIN_A,0);
            digitalWriteFast(CLOCK_PIN_B,0);
            clock_flag = 0;
        }
    }

    if (since_led >= LED_INTERVAL) {
        since_led = 0;
        digitalWriteFast(LED_PIN, 1);
    }

    // get next sensor readings set
    if (since_sensors >= SENSORS_INTERVAL) {
        since_sensors = 0;

        freq_rpm = 0.55 * readFrequencyControls() + 0.45 * freq_rpm;    // smoothing
        freq = freq_rpm / 60.0;
        period_us_prev = period_us;
        period_us = (uint32_t)(period_scale * 1.0 / freq);
        if (abs(period_us - period_us_prev) > 100)   // if there's been a change...
            calcPulse();
    }

    if (since_print >= PRINT_INTERVAL) {
        since_print = 0;
        // diagnostic  sprintf(msg, "  %5lu %5.1f rpm   %6.3f Hz  %7lu us [%5u+%5u  %7lu]\n", (millis()-t0)/1000, freq_rpm, freq, period_us, t_on, t_off, t_on+t_off);
        sprintf(msg, "  %5.1f rpm   %6.2f Hz \n", freq_rpm, freq);
        E(msg);
        //oledPrintMsg(byte x, byte y, char msg[22], int angle);
        u8g2.setFont(u8g2_font_9x15B_mf);  // 11 pix
        sprintf(msg, "%6.1f RPM",freq_rpm);
        oledPrintMsg(10,16, msg, 0);
        u8g2.setFont(u8g2_font_6x12_tr);
        sprintf(msg, "%6.2f Hz",freq);
        oledPrintMsg(31,30, msg, 0);
    }

    if (since_led >= 10) {  // turn off led after N ms (brief flash)
        if (digitalReadFast(LED_PIN) != 0) {
            digitalWriteFast(LED_PIN, 0);
        }
    }


    delayMicroseconds(5);
}

// -- functions -----------------------------------------------------------------------------

/// read 2 pots and convert to a frequency in RPM
float readFrequencyControls() {
    float rpm_coarse = 0;
    float rpm_fine = 0;
    float raw_coarse = (float)readPot(ch_coarse);
    float raw_fine   = (float)readPot(ch_fine);
    float f_rpm;
    float fine_range = 4.0;
    rpm_coarse = mapFloat(raw_coarse, 0, MAX_VAL_ANALOG, MIN_RPM, MAX_RPM);
    rpm_fine   = mapFloat(raw_fine,   0, MAX_VAL_ANALOG, -1*fine_range, fine_range);
    f_rpm = rpm_coarse + rpm_fine;

    return f_rpm;
}

/// read potentiometer
uint16_t readPot(uint8_t ch) {
    uint8_t num_readings = 16;
    uint8_t t_delay_us = 20;
    uint16_t val = readAndAverage(ch, num_readings, t_delay_us);
    return val;
}

/// read N times and average at interval t
uint16_t readAndAverage(uint8_t ch, uint8_t N, uint8_t t_us) {
    int32_t val;
    int32_t val_sum = 0;
    // average N readings
    for (uint8_t i = 0; i < N; i++) {
        val = analogRead(ch);
        val_sum += val;
        delayMicroseconds(t_us);
    }
    return (uint16_t)(val_sum / N);
}


void calcPulse() {
    t_on = (uint32_t)(duty_cycle * (float)period_us);            // in usec
    t_on = constrain(t_on, 2000, 10000);                         // keep strobe narrow
    t_off = period_us - t_on;

}

#if 1
double roundToNumDigits(float x, uint8_t n) {
    int k = floor(x);
    double e = pow(10,n);
    return (float)(k * e) / e;
}
#endif

/// map one numerical span to another with floating point values
double mapFloat(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/// Output text string to either or both serial ports: hardware serial, USB serial
void echoString(String str) {
    // prints msg to selected port(s)
    if (PRINT_FLAG) {
        if (CONSOLE_FLAG) CONSOLE.print(str);    // print to console
        if (HWSERIAL_FLAG) HWSERIAL.print(str);  // print to HW serial port
    }
}

//////////////////////////////////////////////////////////////////////////////////////
/*
  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com   All rights reserved.
*/


//char str[22] = "\0";

const byte SCRN_W = 128;
const byte SCRN_H = 32;

void oledInit() {
    // assign pins for cursor control of SelectionList
    // u8g2.begin(/*Select=*/ A0, /*Right/Next=*/ 5, /*Left/Prev=*/ 9, /*Up=*/ 8, /*Down=*/ 10, /*Home/Cancel=*/ A1); // Arduboy DevKit
    u8g2.begin(/*Select=*/ 7, /*Right/Next=*/ A1, /*Left/Prev=*/ A2, /*Up=*/ 0, /*Down=*/ 1, /*Home/Cancel=*/ 8); // Arduboy 10 (Production)
    u8g2.clearBuffer();          // clear the internal memory
    // u8g2.sendBuffer();
    u8g2.setDrawColor(ON);
    u8g2.setFont(u8g2_font_9x15B_mf);  // 11 pix
    // u8g2.setFont(u8g2_font_6x12_tr);
}


void oledPrintMsg(byte x, byte y, char msg[22], int angle) {   // font preselected
    char s[22] = "\0";
    byte num_chars;
    if (angle == 1 || angle == 90) u8g2.setFontDirection(1);   // 1=90
    else                           u8g2.setFontDirection(0);   // 0= 0
    u8g2.setDrawColor(ON);
    num_chars = sprintf(s, "%s", msg);
    u8g2.drawStr(x, y, s);
    u8g2.sendBuffer();
}

/// EOF
