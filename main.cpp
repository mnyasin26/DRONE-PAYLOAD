#include "config.h"

#include <Wire.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>
#include <JY901.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2

// #define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

unsigned long Clock;
unsigned long DhtTimer;

// sensor_t sensor;

HardwareSerial GPSSerial(1);
SoftwareSerial OpenMVSerial;
SoftwareSerial LoraSerial;

TinyGPSPlus gps;

Adafruit_INA219 ina219;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw;
float temperature;
int humidity;

String suhu, kelembaban;

String timestamp;

const int offset = 7;

String KlasifikasiOpenMV = "0", LAT, LON, ALT, SOG, COG, Arus, Tegangan, Daya;
String ID = "DEV1";

void digitalClockDisplay();
void printDigits(int digits);
sensor_t sensor;

int delayMS;

void setup()
{

    Wire.begin();
    Serial.begin(115200);

    JY901.startIIC();
    LoraSerial.begin(9600, SWSERIAL_8N1, 4, 3, false, 95, 11);
    GPSSerial.begin(9600, SERIAL_8N1, 10, 9);
    OpenMVSerial.begin(9600, SWSERIAL_8N1, 20, 21, false, 95, 11);

    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        while (1)
        {
            delay(10);
        }
    }

    dht.begin();

    
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("°C"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("°C"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("%"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("%"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("%"));
    Serial.println(F("------------------------------------"));


    delayMS = sensor.min_delay / 1000;
    // Set delay between sensor readings based on sensor details.
}

sensors_event_t event;

void loop()
{
    if (millis() - DhtTimer >  delayMS)
    {
        dht.temperature().getEvent(&event);
        /* code */
    }
    
    if ((millis() - Clock) > 1000)
    {
        if (isnan(event.temperature))
        {
            Serial.println(F("Error reading temperature!"));
            temperature = 0;
        }
        else
        {
            Serial.print(F("Temperature: "));
            temperature = event.temperature - 5.7;
            Serial.print(temperature);
            Serial.println(F("°C"));
        }

        // Get humidity event and print its value.
        dht.humidity().getEvent(&event);
        if (isnan(event.relative_humidity))
        {
            Serial.println(F("Error reading humidity!"));
            humidity = 0;
        }
        else
        {
            Serial.print(F("Humidity: "));
            humidity = event.relative_humidity;
            Serial.print(event.relative_humidity);
            Serial.println(F("%"));
        }

        ax = JY901.getAccX();
        ay = JY901.getAccY();
        az = JY901.getAccZ();
        gx = JY901.getGyroX();
        gy = JY901.getGyroY();
        gz = JY901.getGyroZ();
        mx = JY901.getMagX();
        my = JY901.getMagY();
        mz = JY901.getMagZ();
        roll = JY901.getRoll();
        pitch = JY901.getPitch();
        yaw = JY901.getYaw();

        // Serial.println("Yahboom IMU 10 Axis : ");
        // Serial.println(String() + "AX =\t" + JY901.getAccX());
        // Serial.println(String() + "AY =\t" + JY901.getAccY());
        // Serial.println(String() + "AZ =\t" + JY901.getAccZ());
        // Serial.println(String() + "GX =\t" + JY901.getGyroX());
        // Serial.println(String() + "GY =\t" + JY901.getGyroY());
        // Serial.println(String() + "GZ =\t" + JY901.getGyroZ());
        // Serial.println(String() + "MX =\t" + JY901.getMagX());
        // Serial.println(String() + "MY =\t" + JY901.getMagY());
        // Serial.println(String() + "MZ =\t" + JY901.getMagZ());
        // Serial.println(String() + "Roll =\t" + JY901.getRoll());
        // Serial.println(String() + "Pitch =\t" + JY901.getPitch());
        // Serial.println(String() + "Yaw =\t" + JY901.getYaw());
        // Serial.println(String() + "Temp =\t" + JY901.getTemp());

        ALT = JY901.getAltitude();
        // Serial.println(String() + "Altitude =\t" + ALT);
        // Serial.println();

        // Serial.println(F("GPS Module : "));
        if (gps.location.isValid())
        {
            // Serial.println(gps.location.lat(), 9);
            // Serial.println(gps.location.lng(), 9);
            // Serial.println(gps.altitude.meters(), 2);
            LAT = String(gps.location.lat(), 9);
            LON = String(gps.location.lng(), 9);
            SOG = String(gps.speed.kmph());
            COG = String(gps.course.deg());
        }
        else
        {
            LAT = String("0.000000000");
            LON = String("0.000000000");
            SOG = String("0");
            COG = String("0");
        }

        timestamp = "";
        if (gps.time.isValid())
        {
            if (gps.time.age() < 500)
            {
                // set the Time to the latest GPS reading
                setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
                adjustTime(offset * SECS_PER_HOUR);
            }

            if (hour() < 10)
                timestamp += "0";
            timestamp += (String)hour() + ":";
            if (minute() < 10)
                timestamp += "0";
            timestamp += (String)minute() + ":";
            if (second() < 10)
                timestamp += "0";
            timestamp += (String)second();

            // Serial.println("time read successfully: " + (String)gps.time.hour() + ":" + (String)gps.time.minute() + ":" + (String)gps.time.second());
            // digitalClockDisplay();
        }
        else
        {
            timestamp = "00:00:00";
        }

        // Serial.println("time:" + timestamp);
        Arus = ina219.getCurrent_mA();
        Tegangan = ina219.getBusVoltage_V();
        Daya = ina219.getPower_mW();
        // Serial.println(String() + "OpenMVData = " + KlasifikasiOpenMV);
        // Serial.println("");
        suhu = (String)temperature;
        kelembaban = (String)humidity;
        Serial.println(String() + "*," + timestamp + "," + LAT + "," + LON + "," + ALT + "," + SOG + "," + COG + "," + Arus + "," + Tegangan + "," + Daya + "," + KlasifikasiOpenMV + "," + ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + mx + "," + my + "," + mz + "," + roll + "," + pitch + "," + yaw + "," + suhu + "," + kelembaban + ",#");
        LoraSerial.println(String() + "*," + timestamp + "," + LAT + "," + LON + "," + ALT + "," + SOG + "," + COG + "," + Arus + "," + Tegangan + "," + Daya + "," + KlasifikasiOpenMV + "," + ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + mx + "," + my + "," + mz + "," + roll + "," + pitch + "," + yaw + "," + suhu + "," + kelembaban + ",#");

        Clock = millis();
    }
    while (GPSSerial.available() > 0)
    {
        gps.encode(GPSSerial.read());
        yield();
    }
    while (OpenMVSerial.available() > 0)
    {
        char c = OpenMVSerial.read();
        if (c == '0')
        {
            KlasifikasiOpenMV = "0";
        }
        else if (c == '1')
        {
            KlasifikasiOpenMV = "1";
        }
        else
        {
            KlasifikasiOpenMV = "0";
        }
        yield();
    }
    while (LoraSerial.available() > 0)
    {
        Serial.write(LoraSerial.read());
        yield();
    }
    while (Serial.available() > 0)
    {
        GPSSerial.write(Serial.read());
        OpenMVSerial.write(Serial.read());
        yield();
    }
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(":");
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}
