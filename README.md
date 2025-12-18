# Table of Contents

# Introduction

# Example
As you can see the Example is using an Arduino MEGA2560 R3, a few I2C 16x2 Displays and potentiometers for simulating target Value, the measured process variable and the PID-Parts.

## Wokwi Scheme
<img width="1069" height="430" alt="image" src="https://github.com/user-attachments/assets/ef8fe357-8f19-4308-832a-0a4abdcfdb89" />

## Diagram.json
```json
{
  "version": 1,
  "author": "Anonymous maker",
  "editor": "wokwi",
  "parts": [
    { "type": "wokwi-arduino-mega", "id": "mega", "top": 0, "left": 0, "attrs": {} },
    {
      "type": "wokwi-lcd1602",
      "id": "lcd1",
      "top": -137.6,
      "left": 466.4,
      "attrs": { "pins": "i2c", "i2cAddress": "0x27" }
    },
    {
      "type": "wokwi-lcd1602",
      "id": "lcd2",
      "top": 246.4,
      "left": 485.6,
      "attrs": { "pins": "i2c", "i2cAddress": "0x26" }
    },
    { "type": "wokwi-vcc", "id": "vcc2", "top": -152.84, "left": 384, "attrs": {} },
    {
      "type": "wokwi-slide-potentiometer",
      "id": "pot1",
      "top": -119.8,
      "left": 959,
      "attrs": { "travelLength": "30" }
    },
    { "type": "wokwi-vcc", "id": "vcc3", "top": -143.24, "left": 835.2, "attrs": {} },
    { "type": "wokwi-gnd", "id": "gnd1", "top": -38.4, "left": 1218.6, "attrs": {} },
    {
      "type": "wokwi-slide-potentiometer",
      "id": "pot2",
      "top": 245,
      "left": 911,
      "attrs": { "travelLength": "30" }
    },
    { "type": "wokwi-vcc", "id": "vcc4", "top": 211.96, "left": 844.8, "attrs": {} },
    { "type": "wokwi-gnd", "id": "gnd2", "top": 336, "left": 1170.6, "attrs": {} },
    {
      "type": "wokwi-text",
      "id": "text1",
      "top": 211.2,
      "left": 969.6,
      "attrs": { "text": "Setpoint" }
    },
    {
      "type": "wokwi-text",
      "id": "text2",
      "top": 211.2,
      "left": 604.8,
      "attrs": { "text": "Setpoint" }
    },
    {
      "type": "wokwi-text",
      "id": "text3",
      "top": -192,
      "left": 528,
      "attrs": { "text": "Measured Process Variable" }
    },
    {
      "type": "wokwi-text",
      "id": "text4",
      "top": -192,
      "left": 940.8,
      "attrs": { "text": "Measured Process Variable" }
    },
    {
      "type": "wokwi-lcd1602",
      "id": "lcd3",
      "top": 54.4,
      "left": 476,
      "attrs": { "pins": "i2c", "i2cAddress": "0x25" }
    },
    {
      "type": "wokwi-text",
      "id": "text5",
      "top": 19.2,
      "left": 595.2,
      "attrs": { "text": "PID Output" }
    },
    {
      "type": "wokwi-lcd1602",
      "id": "lcd4",
      "top": 16,
      "left": -330.4,
      "attrs": { "pins": "i2c", "i2cAddress": "0x24" }
    },
    { "type": "wokwi-text", "id": "text6", "top": -153.6, "left": -768, "attrs": { "text": "P" } },
    {
      "type": "wokwi-slide-potentiometer",
      "id": "pot3",
      "top": -124.6,
      "left": -783,
      "rotate": 180,
      "attrs": { "travelLength": "30" }
    },
    {
      "type": "wokwi-slide-potentiometer",
      "id": "pot4",
      "top": 48.2,
      "left": -783,
      "rotate": 180,
      "attrs": { "travelLength": "30" }
    },
    {
      "type": "wokwi-slide-potentiometer",
      "id": "pot5",
      "top": 240.2,
      "left": -783,
      "rotate": 180,
      "attrs": { "travelLength": "30" }
    },
    { "type": "wokwi-vcc", "id": "vcc1", "top": -162.44, "left": -528, "attrs": {} },
    { "type": "wokwi-gnd", "id": "gnd3", "top": 364.8, "left": -903, "attrs": {} },
    { "type": "wokwi-gnd", "id": "gnd4", "top": 124.8, "left": -403.8, "attrs": {} },
    { "type": "wokwi-vcc", "id": "vcc5", "top": -162.44, "left": -422.4, "attrs": {} },
    { "type": "wokwi-text", "id": "text7", "top": 38.4, "left": -768, "attrs": { "text": "I" } },
    { "type": "wokwi-text", "id": "text8", "top": 220.8, "left": -768, "attrs": { "text": "D" } }
  ],
  "connections": [
    [ "lcd1:GND", "mega:GND.1", "black", [ "h0" ] ],
    [ "lcd2:GND", "mega:GND.2", "black", [ "h0" ] ],
    [ "vcc2:VCC", "lcd1:VCC", "red", [ "v0" ] ],
    [ "lcd1:SDA", "mega:20", "green", [ "h0" ] ],
    [ "lcd1:SCL", "mega:21", "orange", [ "h0" ] ],
    [ "pot1:SIG", "mega:A0", "green", [ "h-28.8", "v66.4", "h393.6", "v480", "h-1039.5" ] ],
    [ "vcc3:VCC", "pot1:VCC", "red", [ "v0" ] ],
    [ "gnd1:GND", "pot1:GND", "black", [ "v0" ] ],
    [ "vcc4:VCC", "pot2:VCC", "red", [ "v48", "h48" ] ],
    [ "gnd2:GND", "pot2:GND", "black", [ "v-48", "h-62.8" ] ],
    [ "pot2:SIG", "mega:A1", "green", [ "h-57.6", "v162.4", "h-633.6" ] ],
    [ "lcd2:VCC", "lcd1:VCC", "red", [ "h-48", "v-383.9" ] ],
    [ "lcd2:SDA", "lcd1:SDA", "green", [ "h-67.2", "v-383.8" ] ],
    [ "lcd2:SCL", "lcd1:SCL", "orange", [ "h-86.4", "v-383.7" ] ],
    [ "lcd3:GND", "lcd2:GND", "black", [ "h-28.8", "v192" ] ],
    [ "lcd3:VCC", "lcd1:VCC", "red", [ "h-38.4", "v-191.9" ] ],
    [ "lcd3:SDA", "lcd2:SDA", "green", [ "h-57.6", "v192.2" ] ],
    [ "lcd3:SCL", "lcd2:SCL", "orange", [ "h-76.8", "v192.3", "h86.4" ] ],
    [ "vcc1:VCC", "pot3:VCC", "red", [ "v0" ] ],
    [ "pot4:VCC", "vcc1:VCC", "red", [ "h57.6", "v-249.6" ] ],
    [ "pot5:VCC", "vcc1:VCC", "red", [ "h0" ] ],
    [ "gnd3:GND", "pot5:GND", "black", [ "v0" ] ],
    [ "pot4:GND", "gnd3:GND", "black", [ "h-110.8", "v249.6" ] ],
    [ "pot3:GND", "gnd3:GND", "black", [ "v0", "h-101.2" ] ],
    [ "pot3:SIG", "mega:A8", "green", [ "h124.8", "v471.2", "h729.6" ] ],
    [ "pot4:SIG", "mega:A9", "green", [ "h115.2", "v308", "h768" ] ],
    [ "pot5:SIG", "mega:A10", "green", [ "h105.6", "v125.6", "h777.6" ] ],
    [ "lcd4:GND", "gnd4:GND", "black", [ "h0" ] ],
    [ "lcd4:VCC", "vcc5:VCC", "red", [ "h0" ] ],
    [ "lcd4:SDA", "mega:20", "green", [ "h-9.6", "v-95.8", "h662.4" ] ],
    [ "lcd4:SCL", "mega:21", "orange", [ "h-19.2", "v-114.9", "h681.6" ] ]
  ],
  "dependencies": {}
}
```

## Code
```c++
#include <pidController.hpp>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

constexpr static uint8_t ADDR1 = 0x27; // I2C address for the first LCD
constexpr static uint8_t ADDR2 = 0x26; // I2C address for the second LCD
constexpr static uint8_t ADDR3 = 0x25; // I2C address for the second LCD
constexpr static uint8_t ADDR4 = 0x24; // I2C address for the second LCD
constexpr static uint8_t COLS = 16;    // Number of columns for the LCD
constexpr static uint8_t ROWS = 2;     // Number of rows for the LCD
constexpr static uint8_t P = A8;
constexpr static uint8_t I = A9;
constexpr static uint8_t D = A10;

constexpr static uint8_t PROCESS_VARIABLE = A0;
constexpr static uint8_t SETPOINT = A1;

LiquidCrystal_I2C lcd1(ADDR1, COLS, ROWS);
LiquidCrystal_I2C lcd2(ADDR2, COLS, ROWS);
LiquidCrystal_I2C lcd3(ADDR3, COLS, ROWS);
LiquidCrystal_I2C lcd4(ADDR4, COLS, ROWS);

auto writeFirstLine(LiquidCrystal_I2C &lcd, const String &text) -> void;
auto writeSecondLine(LiquidCrystal_I2C &lcd, const String &text) -> void;
auto mapf(float x, float in_min, float in_max, float out_min, float out_max) -> float;

PID _PID;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(10); // Wait for serial to initialize

  lcd1.init();
  lcd1.backlight();
  delay(10);  
  lcd2.init();
  lcd2.backlight();
  delay(10);
  lcd3.init();
  lcd3.backlight();
  delay(10);
  lcd4.init();
  lcd4.backlight();

  _PID.setDirection(PID::Direction::REVERSE);
}

void loop()
{
 
  double pv = analogRead(PROCESS_VARIABLE);
  double sp = analogRead(SETPOINT);

  
  auto p = mapf(analogRead(P), 0, 1023, 0.0f, 10.0f);
  auto i = mapf(analogRead(I), 0, 1023, 0.0f, 10.0f);
  auto d = mapf(analogRead(D), 0, 1023, 0.0f, 10.0f);

  _PID.setTunings(static_cast<double>(p), static_cast<double>(i), static_cast<double>(d), 1000.0);

  // Calculate PID output
  auto output = _PID.calc(sp, pv);

  writeFirstLine(lcd1, "Process Var:");
  writeSecondLine(lcd1, String(pv, 2));

  writeFirstLine(lcd2, "Setpoint:");
  writeSecondLine(lcd2, String(sp, 2));

  writeFirstLine(lcd3, "PID Output:");
  writeSecondLine(lcd3, String(output, 2));

  writeFirstLine(lcd4, "P / I / D:");
  writeSecondLine(lcd4, String(p) + " / " + String(i) + " / " + String(d));
}

auto writeFirstLine(LiquidCrystal_I2C &lcd, const String &text) -> void
{
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print(text);
}

auto writeSecondLine(LiquidCrystal_I2C &lcd, const String &text) -> void
{
  lcd.setCursor(0, 1); 
  lcd.print(text);
}
auto mapf(float x, float in_min, float in_max, float out_min, float out_max) -> float
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```

# Functions
The following chapter describes the public functions which you can use to run the class



