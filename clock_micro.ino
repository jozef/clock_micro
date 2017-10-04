#include <Wire.h>
#include <OneWire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "melody.h"
#include "LowPower.h"
#include <internalVCC.h>

// hearth beat var
byte onboard_led = 13;
byte loop_tick = 0;

// button
byte buttonPin = 1;
byte buttonState = 0;
byte button_irq = 3;

// real time clock vars
RTC_DS1307 rtc;  // RTClib clock
DateTime datetime_boot;
char* day_names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char* month_names[] = {"-", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
byte hearth_beat_status = HIGH;
byte sqw_tick_ping = 1;
const byte rtcsqw_pin = 0;
const byte rtcsqw_irq = 2;
uint32_t uptime = 0;

// display vars
LiquidCrystal_I2C lcd(0x3f, 20, 4);    // 20x4 display with I2C address 0x3f
byte keep_lcd_on = 10;

// temperature vars
#define ONE_WIRE_BUS 8    // data one wire port 7
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float cur_temp;
float min_temp;
float max_temp;
uint8_t temp_sensor_current = 0;
uint8_t temp_sensors_count = 0;
DeviceAddress temp_sensors[5];

void setup () {
    pinMode(onboard_led, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);

    Serial.begin(9600);

    // display init
    lcd.begin();
    lcd.clear();
    lcd.backlight();      // lcd.noBacklight();
    lcd.setCursor(0,3);
    lcd.print("starting up");

    // rtc init
    //// rtc.adjust(DateTime(2017, 3, 26, 21, 37, 00)); // must be set one time if needed
    if (! rtc.begin()) {
        Serial.println("Clock not connected!");
        while (1) { delay(1000); };
    }
    if (! rtc.isrunning()) {
        Serial.println("Clock not running, setting now...");
        rtc.adjust(DateTime(1970, 2, 26, 00, 00, 00));
    }
    //rtc.adjust(DateTime(2017, 7, 20, 21, 43, 00));
    datetime_boot = rtc.now();
    pinMode(rtcsqw_pin, INPUT_PULLUP);
    //pinMode(rtcsqw_pin, INPUT);
    attachInterrupt(rtcsqw_irq, sqw_tick, RISING);   //attachInterrupt(digitalPinToInterrupt(rtcsqw_pin), sqw_tick, RISING);
    rtc.writeSqwPinMode(SquareWave1HZ);

    // button
    attachInterrupt(button_irq, button_push, LOW);   //attachInterrupt(digitalPinToInterrupt(button_pin), button_push, LOW);

    // one wire sensort init
    sensors.begin();
    temp_sensors_count = sensors.getDeviceCount();
    if (temp_sensors_count) {
        if (temp_sensors_count > 5) {
            temp_sensors_count = 5;
        }
        for (uint8_t i = 0;i<temp_sensors_count;i++) {
            sensors.getAddress(temp_sensors[i],i);
            sensors.setResolution(temp_sensors[i], TEMPERATURE_PRECISION);
        }
        sensors.requestTemperaturesByAddress(temp_sensors[temp_sensor_current]);
        min_temp = sensors.getTempC(temp_sensors[temp_sensor_current]);
        max_temp = min_temp;
        showTemperature();
    }

    // just in case....
    lcd.setCursor(0,3);
    lcd.print("delay 3s");
    delay(3000);
    showDateTime(datetime_boot, 0);
}

void loop () {
    if (buttonState) {
        buttonState = 0;
        keep_lcd_on = 10;
        lcd.backlight();

        lcd.setCursor(0,3);
        lcd.print("music! ");
        playTune(melody, durations, tempo);
    }

    // do every new second
    if (sqw_tick_ping) {
        sqw_tick_ping = 0;
        uptime++;

        if (keep_lcd_on--) {
            if (!keep_lcd_on) {
                lcd.noBacklight();
            }
        }

        // Serial.println("uptime: "+String(int(millis()/1000) % 60));

        // read temperature every 60s
        // read voltage every 60s
        if (uptime % 60 == 0) {
            DateTime datetime_now = rtc.now();
            TimeSpan uptime_delta = datetime_now - datetime_boot;
            uptime = uptime_delta.totalseconds();
            uint32_t epoch = datetime_now.unixtime();
            showDateTime(datetime_now, uptime_delta);
            showTemperature();
            lcd.setCursor(14,3);
            lcd.print(vccToString(vccVoltage()));
        }
    }

    // start powersaveing (power-down) after first minute
    if (uptime >= 60) {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        //hearth_beat_status = !hearth_beat_status;
        //digitalWrite(onboard_led, hearth_beat_status);
    }
    else {
        digitalWrite(onboard_led, HIGH);
        delay(50);
        digitalWrite(onboard_led, LOW);
        delay(200);
    }
    // start powersaveing (idle) after 1min
    //else if (uptime > 60) {
    //    // for ATmega32U4
    //    LowPower.idle(
    //        SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,
    //        TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF
    //    );
    //}

    // hearth beat every 7 iterations (in powersave mode every 7s)
    //if ((loop_tick++ % 7 == 0) || (hearth_beat_status == HIGH)) {
    //    hearth_beat_status = !hearth_beat_status;
    //    digitalWrite(onboard_led, hearth_beat_status);
    //}

    // on serial
    //int serial_data_size = Serial.available();
    //if (serial_data_size) {
    //    String readed = Serial.readString();
    //    // Serial.println("received: "+readed);
    //    // playTune(melody, durations, tempo);
    //}
}

void sqw_tick () {
    sqw_tick_ping = 1;
}

void button_push () {
    buttonState = !digitalRead(buttonPin);
}

void showDateTime (DateTime now, TimeSpan uptime_delta) {
    lcd.setCursor(0,0);
    lcd.print(formatDate(now));
    lcd.setCursor(0,1);
    lcd.print(formatTime(now));
    lcd.setCursor(0,3);
    lcd.print("since: "+upTimeString(now, uptime_delta)+"   ");
}

void showTemperature () {
    if (!temp_sensors_count) {
        return;
    }

    sensors.requestTemperaturesByAddress(temp_sensors[temp_sensor_current]);
    cur_temp = sensors.getTempC(temp_sensors[temp_sensor_current]);
    if (cur_temp > max_temp) {
        max_temp = cur_temp;
    }
    if (cur_temp < min_temp) {
        min_temp = cur_temp;
    }

    lcd.setCursor(10,1);
    lcd.print(
        tempToString(cur_temp)
        +String(char(0xDF))     // °
        +String("C")
        +"   "
    );
    lcd.setCursor(0,2);
    lcd.print(
        "["+String(temp_sensor_current+1)+"/"+String(temp_sensors_count)+"]"
        +" ("
        +tempToString(min_temp)
        +"/"
        +tempToString(max_temp)
        +")  "
    );

    // rotate temperature sensors
    temp_sensor_current++;
    if (temp_sensor_current >= temp_sensors_count ) {
        temp_sensor_current = 0;
    }
}

String timeNumToString (uint8_t num) {
    if (num < 10) {
        return String("0"+String(num));
    }
    else {
        return String(num);
    }
}

String formatDate (DateTime dt) {
    return String(
        String(day_names[dt.dayOfTheWeek()])
        +' '+
        timeNumToString(dt.day())
        +' '+
        String(month_names[dt.month()])
        +' '+
        String(dt.year())
    );
}

String formatTime (DateTime dt) {
    return String(
        timeNumToString(dt.hour())
        +':'+
        timeNumToString(dt.minute())
//        +':'+
//        timeNumToString(dt.second())
    );
}

String tempToString (float temp) {
    int degree = int(temp);
    int degree_fraction = int((temp - (float)degree)*10);
    return String(
        String(degree)
        +"."
        +String(degree_fraction)
    );
}

String vccToString (float vcc_in) {
    int volts = int(vcc_in / 1000);
    String mvolts = String(int(vcc_in - (volts * 1000)));
    while (mvolts.length() < 3) {
        mvolts = "0" + mvolts;
    }
    return String(
        String(volts)
        +"."
        +String(mvolts)
        +String("V")
    );
}

String upTimeString (DateTime now, TimeSpan uptime_delta) {
    String upTimeStr;

    if (uptime_delta.days() > 0) {
        return String(uptime_delta.days())+"d";
    }
    else if (uptime_delta.hours() > 0) {
        return String(uptime_delta.hours())+"h";
    }
    else if (uptime_delta.minutes() > 0) {
        return String(uptime_delta.minutes())+"m";
    }

    return String(uptime_delta.seconds())+"s";
}
