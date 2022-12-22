from everywhereml.arduino import Sketch, Ino, H
from datetime import datetime

sketch = Sketch(name="RTC")
date = datetime.utcnow()

code = '''#include <DS3231.h>
#include <Wire.h>

DS3231 Clock;

void setup() {{
    delay(1000);
    Serial.begin(115200);
    Wire.begin();
    Clock.setSecond({});
    Clock.setMinute({});
    Clock.setHour({});
    Clock.setDoW({});
    Clock.setDate({});
    Clock.setMonth({});
    Clock.setYear({});
}}

void loop() {{
}}'''.format(date.second+5,
             date.minute,
             date.hour,
             date.weekday()+1,
             date.day,
             date.month,
             date.year)

sketch += Ino(code)

if sketch.compile(board='Arduino Pro or Pro Mini').is_successful:
    print('Log', sketch.output)
    print('Sketch stats', sketch.stats)
else:
    print('ERROR', sketch.output)

sketch.upload(port='/dev/ttyUSB0')
print(sketch.output)
