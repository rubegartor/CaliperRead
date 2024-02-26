
# Dial / Caliper Reader

Example of digital dial/caliper indicator reader implemented on an ESP32 with non-blocking code

It will output though Serial connection the result in mm (respect the International System of Units, fuck the inches)

## How it works 🚀  
To carry out the tests, the following material was used:
- Neoteck Indicador de Dial Digital DTI 0.01mm [https://www.amazon.es/dp/B08B183RSK](https://www.amazon.es/dp/B08B183RSK)
- InnoMaker LA1010 USB Logic Analyzer [https://www.amazon.es/InnoMaker-LA1010-Analyzer-Channels-Instrument/dp/B07D21GG6J](https://www.amazon.es/InnoMaker-LA1010-Analyzer-Channels-Instrument/dp/B07D21GG6J)
- ESP32 DevKit


### Logic Analyzer tests
0.00mm test:

![0.0mm](https://i.imgur.com/ahKTgQi.png)  

1.75mm test:
![0.0mm](https://i.imgur.com/X3nZsKg.png)

Time between readings:
![timing](https://i.imgur.com/Mbzo8ct.png)

### How to decode bit buffer:

First, we need to convert the pulses into a sequence of bits (first bit of sequence is ignored), then a for loop is used to loop through each element of the buff array.
If the value at the current position of the buff array is equal to 1 (HIGH), the corresponding bit is set to val using the bit shift operator (<<) and the bitwise OR operator (|). For negative readings, the bit in position 20 is used, if it is equal to 1 (HIGH) it will be a negative value.

![1.75_draw](https://i.imgur.com/ceMNqk1.png)

```
// Code to perform manual calculation

#define BUFFER_SIZE 23

uint8_t buff[BUFFER_SIZE] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void setup() {
  Serial.begin(115200);

  delay(1000); 

  int16_t val = 0;

  for (uint8_t i; i < BUFFER_SIZE; i++) {
    if (buff[i] == 0x1) {
      val |= 1 << i;
    }
  }

  Serial.println(val / 100.00);
}

void loop() {}
```

## Authors ✒️

* **Rubén García** - [rubegartor](https://github.com/rubegartor)

## License 📄

This project is under the GPL-3.0 license - [LICENSE.MD](LICENSE.MD) for more details
