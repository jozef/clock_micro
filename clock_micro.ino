#include <Wire.h>
#include <OneWire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

// hearth beat var
int onboard_led = 13;

// real time clock vars
RTC_DS1307 rtc;  // RTClib clock
DateTime datetime_boot;
char* day_names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char* month_names[] = {"-", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
int hearth_beat_status = HIGH;
byte sqw_tick_ping = 1;
const byte rtcsqw_pin = 7;
const byte rtcsqw_irq = 4;

// display vars
LiquidCrystal_I2C lcd(0x3f, 20, 4);    // 20x4 display with I2C address 0x3f

// temperature vars
#define ONE_WIRE_BUS 8    // data one wire port 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float min_temp;
float max_temp;

void setup () {
    pinMode(onboard_led, OUTPUT);

    Serial.begin(9600);

    // display init
    lcd.begin();
    lcd.clear();
    lcd.backlight();      // lcd.noBacklight();
    lcd.setCursor(0,3);
    lcd.print("starting up...");

    // rtc init
    //// rtc.adjust(DateTime(2017, 3, 26, 21, 37, 00)); // must be set one time if needed
    if (! rtc.begin()) {
        Serial.println("Clock not connected!");
        while (1) { delay(1000); };
    }
    if (! rtc.isrunning()) {
        Serial.println("Clock not running, setting now...");
        rtc.adjust(DateTime(1970, 2, 26, 01, 05, 00));
    }
    datetime_boot = rtc.now();
    pinMode(rtcsqw_pin, INPUT_PULLUP);
    attachInterrupt(rtcsqw_irq, sqw_tick, RISING);   //attachInterrupt(digitalPinToInterrupt(rtcsqw_pin), sqw_tick, RISING);
    rtc.writeSqwPinMode(SquareWave1HZ);

    // one wire sensort init
    sensors.begin();
    sensors.requestTemperatures();
    min_temp = sensors.getTempCByIndex(0);
    max_temp = min_temp;
}

void loop () {
    // do every new second
    if (sqw_tick_ping) {
        sqw_tick_ping = 0;
        DateTime datetime_now = rtc.now();

        // update time every second
        showDateTime(datetime_now);

        // hearth beat
        uint32_t epoch = datetime_now.unixtime();

        // hearth beat every 3s
        if (epoch % 3 == 0) {
            hearth_beat_status = !hearth_beat_status;
            digitalWrite(onboard_led, hearth_beat_status);
        }

        // read temperature every 5s
        if (epoch % 5 == 0) {
            showTemperature();
        }

        // rotate three texts every 7s
        if (epoch % 7 == 0) {
            lcd.setCursor(0,2);
            byte one_of_three = epoch / 7 % 2;
            switch (one_of_three) {
                case 0:
                    lcd.print(
                        "max: "
                        +tempToString(max_temp)
                        +"  "
                    );
                    break;
                case 1:
                    lcd.print(
                        "min: "
                        +tempToString(min_temp)
                        +"    "
                    );
                    break;
            }
        }
    }
}

void sqw_tick () {
    sqw_tick_ping = 1;
}

void showDateTime (DateTime now) {
    lcd.setCursor(0,0);
    lcd.print(formatDate(now));
    lcd.setCursor(0,1);
    lcd.print(formatTime(now));
    lcd.setCursor(0,3);
    lcd.print("since: "+upTime(datetime_boot)+"     ");
}

void showTemperature () {
    sensors.requestTemperatures();
    float cur_temp = sensors.getTempCByIndex(0);
    if (cur_temp > max_temp) {
        max_temp = cur_temp;
    }
    if (cur_temp < min_temp) {
        min_temp = cur_temp;
    }

    lcd.setCursor(10,1);
    lcd.print(
        tempToString(cur_temp)
        +String("  ")
    );
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
        +':'+
        timeNumToString(dt.second())
    );
}

String tempToString (float temp) {
    int degree = int(temp);
    int degree_fraction = int((temp - (float)degree)*10);
    return String(
        String(degree)
        +"."
        +String(degree_fraction)
        +String(char(0xDF))     // °
        +String("C")
    );
}

String upTime (DateTime since) {
    String upTimeStr;
    TimeSpan delta = rtc.now() - since;

    if (delta.days() > 0) {
        return String(delta.days())+"d";
    }
    else if (delta.hours() > 0) {
        return String(delta.hours())+"h";
    }
    else if (delta.minutes() > 0) {
        return String(delta.minutes())+"m";
    }

    return String(delta.seconds())+"s";
}
