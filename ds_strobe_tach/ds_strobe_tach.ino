/*  Use Teensy to make a strobe tachometer
 *  Use adjustable input (dual pot) to set a frequency in RPM
 *    display frequency
 *    flash LEDs in short pulse for strobe
 *
 *  See PWM and resolution at https://www.pjrc.com/teensy/td_pulse.html
 *  be sure to use PWM-able pins 3 4 5 6 9... on T3.x
 *
 *  To generate pulse, use low-freq PWM method (couldn't get to work; changed to calculated software pulse
 *
*/

#define HWSERIAL Serial1
#define CONSOLE  Serial                 // USB port debug console (IDE)
#define E        echoString             // my little print shortcut

/////////////////////////////////////////////////////////////////////////////////////
const String VERSION_NUM = "1.0.1";     // increased to 0.1rpm resolution
const bool PRINT_FLAG = 1;              // enable print of streaming data (in echoString)
const bool HWSERIAL_FLAG = 0;           // send msgs to hardware serial port
const bool CONSOLE_FLAG = 1;            // send msgs to standard USB output port
/////////////////////////////////////////////////////////////////////////////////////

byte CLOCK_PIN_A = 5;
byte CLOCK_PIN_B = 6;
byte LED_PIN = 13;
byte ch_coarse = A7;
byte ch_fine = A8;
const float MAX_VAL_ANALOG = 65535.0;
const float MAX_RPM = 400;  // sets range of adjustment
// intervals for various timed events
const uint16_t SENSORS_INTERVAL = 100;      // ms
const uint16_t PRINT_INTERVAL = 200;        // ms
const uint16_t LED_INTERVAL = 1000;         // ms
char msg[80];
float f_freq = 1.00;     // very slow
float f_rpm = f_freq * 60;
uint16_t t_on;
uint16_t t_off;
uint32_t t0;
bool clock_flag = 0;
// timer variables
elapsedMillis since_sensors = 0;        // ms
elapsedMillis since_print = 0;          // ms
elapsedMillis since_led = 0;            // ms
elapsedMillis since_pulse = 0;          // ms

void setup() {
    pinMode(CLOCK_PIN_A, OUTPUT);
    pinMode(CLOCK_PIN_B, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    // LED startup splash
    for (int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH);    // turn the LED on (HIGH is the voltage level)
        digitalWrite(CLOCK_PIN_A, HIGH);  // turn the LED on (HIGH is the voltage level)
        delay(50);                      // wait for a time
        digitalWrite(LED_PIN, LOW);     // turn the LED off by making the voltage LOW
        digitalWrite(CLOCK_PIN_A, LOW);   // turn the LED off by making the voltage LOW
        delay(300);                     // wait for a time
    }
    digitalWrite(LED_PIN,1);
    analogWriteFrequency(CLOCK_PIN_A, 50000);    // set PWM freq
    analogReadResolution(16);
    analogWriteResolution(8);  //

    // check PWM change in brightness
    analogWrite(CLOCK_PIN_A,127);   // 50%
    delay(250);
    analogWrite(CLOCK_PIN_A,64);   // 100%
    delay(250);
    analogWrite(CLOCK_PIN_A,10);    // 10%
    delay(250);
    analogWrite(CLOCK_PIN_A,0);     // 0%
    delay(250);
    // print header message
    E("Strobe Tachometer \n");
    E("Version " + VERSION_NUM + "\n");

    t0 = millis();
}

// --- main loop --------------------------------------------------------------------------------
void loop() {
    // get next sensor readings set

    static float v_coarse = 0;
    static float v_fine   = 0;
    float f_period = 1000;
    static float duty_cycle = 0.05;             // 0.10 = 10%
    static float f_rpm_prev = 0;
    if (since_sensors >= SENSORS_INTERVAL) {
        since_sensors = 0;

        v_coarse = (MAX_RPM*(float)readPot(ch_coarse) / MAX_VAL_ANALOG);
        v_fine   =  (MAX_RPM/100*(float)readPot(ch_fine) / MAX_VAL_ANALOG - MAX_RPM/100/2);
        f_rpm_prev = f_rpm;
        f_rpm = (float)(0.1*((int)(10*(v_coarse + v_fine))));
        f_rpm = constrain(f_rpm,10,5000);
        f_freq = f_rpm / 60.0;
        f_period = 1.0 / f_freq * 1000;
        if(f_period < 5) E("LOW ");  // temp
        if (abs(f_rpm - f_rpm_prev) > 0) {
            t_on = (int)(duty_cycle * f_period);            // in msec
            t_on = constrain(t_on,2,10);                           // keep strobe narrow
            t_off = (int)(f_period) - t_on +(f_period - (int)f_period < 0.5);       // add one count if truncated
        }
    }

    if (since_print >= PRINT_INTERVAL) {
        since_print = 0;
        //sprintf(msg, "  %6.1frpm %6.2fHz  C=%5.1f F=%5.1f\n",f_rpm, f_freq, v_coarse, v_fine);
        sprintf(msg, "  %5lu %5.1f rpm   %6.3f Hz  %6.1f ms [%5u %5u]\n", (millis()-t0)/1000, f_rpm, f_freq, f_period, t_on, t_off);
        E(msg);
    }

    if (since_led >= LED_INTERVAL) {
        since_led = 0;
        digitalWriteFast(LED_PIN, 1);
    }

    if (since_led >= 10) {  // turn off led after N ms (brief flash)
        if (digitalReadFast(LED_PIN) != 0) {
            digitalWriteFast(LED_PIN, 0);
        }
    }

    if (since_pulse >= (t_on + t_off)) {
        since_pulse = 0;
        if (clock_flag == 0) {
            digitalWriteFast(CLOCK_PIN_A,1);
            digitalWriteFast(CLOCK_PIN_B,1);
            clock_flag = 1;
        }
    }
    if (since_pulse >= t_on) {
        if (clock_flag == 1) {
            digitalWriteFast(CLOCK_PIN_A,0);
            digitalWriteFast(CLOCK_PIN_B,0);
            clock_flag = 0;
        }
    }
    delayMicroseconds(200);
}

// -- functions -----------------------------------------------------------------------------

/// read potentiometer
uint16_t readPot(uint8_t ch) {
    uint8_t num_readings = 16;
    uint8_t t_delay_us = 2;
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


/// Output text string to either or both serial ports: hardware serial, USB serial
void echoString(String str) {
    // prints msg to selected port(s)
    if (PRINT_FLAG) {
        if (CONSOLE_FLAG) CONSOLE.print(str);    // print to console
        if (HWSERIAL_FLAG) HWSERIAL.print(str);  // print to HW serial port
    }
}


/// EOF
