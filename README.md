# MPU6050_I2C_library
Bibliotek til sampling med MPU6050 IMU

## Inkluder MPU6050 bibliotek i PlatformIO
Kopier ```I2Cdev``` og ```MPU6050``` mappen til ```lib``` mappen, lavet af PlatformIO. TIlføj det følgende til toppen af ```main.cpp``` filen i mappen ```src```:
```C
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
```
MPU6050 biblioteket er nu klar til brug.

## Brug af MPU6050 biblioteket
For at bruge MPU6050 biblioteket, skal ```Wire```  biblioteket først initialiseres ved at tilføje det følgende setup:
```C++
void setup() {
	...
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	...
}
```
Nu initialiseres MPU6050 klassen, ved at tilføje det følgende til toppen af ```main.cpp``` filen. Samtidig klargøres variabler til acclerometer- og gyroskopdata:
```C++
MPU6050 accelgyro; // Initialize MPU6050 class.
int16_t ax, ay, az; // Accelerometer data variables
int16_t gx, gy, gz; // Gyroscope data variables
... //other code
void setup() {
	... //other code
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	... //other code
}
```
Initialiser nu MPU6050 klassen ved at klade ```initialize()``` funktionen i setup:
```C++
MPU6050 imu; // Initialize MPU6050 class.
int16_t ax, ay, az; // Accelerometer data variables
int16_t gx, gy, gz; // Gyroscope data variables
... //other code
void setup() {
	... //other code
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	imu.initialize(); // Initialize the MPU6050
	... //other code
}
```
For at hente acclerometer- og gyroskopdata kan der nu blot køres: 
```C++
imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Get accelerometer and gyroscope data
```
## Brugbare funktioner
### Test forbindelse
For at teste forbindelsen til MPU6050:
```C++
imu.testConnection() ; // Test MPU6050 connection
```

### Sæt og hent samplerate
For at sætte og hente samplerate for gyroskopet (accelerometer sample rate er fast):
```C++
uint8_t rate = imu.getRate();
imu.setRate(rate); // Takes a uint8_t as input
```
Hvor rate kan være et heltal fra 0 til 7. Basis sampleraten af gyroskopet (GFS) er 8kHz når det digitale lavpasfilter er slået fra (DLPF, se næste sektion) og 1 kHz når DLPF er aktivt. Sampleraten af gyroskopet kan findes ved:
```
Samplerate = GFS / (1 + rate)
```

### Aktiver digital lavpasfilter
For at aktivere eller hente konfiguration af det digitale lavpasfilter anvendes følgedne funktioner:
```C++
uint8_t getDLPFMode();
void setDLPFMode(uint8_t bandwidth);
```
Hvor bandwidth er et heltal mellem 0 til 6 som giver følgende konfiguration af lavpasfilteret:
| DLPF int | Accl bandwidth | Accl delay | Gyro bandwidth | Gyro delay | Gyro sample rate |
| ---      | ---            | ---        | ---             | ---        | ---              |
| 0        | 260 Hz         | 0 ms       | 256 Hz          | 0.98 ms    | 8 kHz            |
| 1        | 184 Hz         | 2.0 ms     | 188 Hz          | 1.9 ms     | 1 kHz            |
| 2        | 94 Hz          | 3.0 ms     | 98 Hz           | 2.8 ms     | 1 kHz            |
| 3        | 44 Hz          | 4.9 ms     | 42 Hz           | 4.8 ms     | 1 kHz            |
| 4        | 21 Hz          | 8.5 ms     | 20 Hz           | 8.3 ms     | 1 kHz            |
| 5        | 10 Hz          | 13.8 ms    | 10 Hz          | 13.4 ms    | 1 kHz            |
| 6        | 5 Hz           | 19.0 ms    | 5 Hz           | 18.6 ms    | 1 kHz            |

### Aktiver digital højpasfilter

```C++
uint8_t getDHPFMode();
void setDHPFMode(uint8_t mode);
```

### Set gyro range
For at indstille eller hente rangen på gyroskopet anvendes følgende funktioner:
```C++
uint8_t getFullScaleGyroRange();
void setFullScaleGyroRange(uint8_t range);
```

### Set accl range

```C++
uint8_t getFullScaleAccelRange();
void setFullScaleAccelRange(uint8_t range);
```

## Test funktioner
For at bruge testfunktionerne skal følgende bibliotek inkluderes i toppen af ```main.cpp``` filen: 
```C++
#include <MPU6050_testFunc.h>
```
Følgende funktioner er tilgængelige:
```C++
// Test sample tid ved hentning af acclerometerdata
void MPU6050testSampleTimeAccl(MPU6050 *imuInstance);
// Test sample tid ved hentning af gyroskopdata
void MPU6050testSampleTimeGyro(MPU6050 *imuInstance);
// Test sample tid ved hentning af acclerometer- og gyroskopdata
void MPU6050testSampleTime6D(MPU6050 *imuInstance);
```

## Sådan virker bibliotekerne
MPU6050 er et digitalt IMU, som tilgåes via I2C protokollen. Derfor bruges to biblioteker: ```I2Cdev``` biblioteket og ```MPU6050``` biblioteket. 

### I2Cdev biblioteket 
```I2Cdev``` biblioteket indeholder funktioner til at lave forbindelse, læse og skrive til enheder på en I2C bus. Biblioteket indeholder hovedesageligt funktioner til at læse og skrive bits, bytes og words fra og til I2C bussen. Et eksempel på dette er følgende funktioner (fra ```I2Cdev.h``` filen):
```C++
// Read bits from I2C bus
static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
// Write bits to the I2C bus
static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
```
Bag scenen, virker disse funktioner ved at implementere ```Wire``` bibloiteket, hvilket er er standard bibliotek i C. ```Wire``` bibloiteket bruges ved først at vælge den addresse som den enhed der skal kommunikeres til har på I2C bussen, her kaldes ```devAddr```. Denne addresse vælges i ```Wire``` bibloiteket ved at kalde funktionen ```Wire.beginTransmission(devAddr);```. Herefter defineres det data som ønskes kommunikeret til enheden. Dette data er ofte en addresse til et register i enheden som enten skal skrives til eller læses fra. Addressen til registeret kaldes ```regAddr```. Data der skal sendes defineres ved funktionen ```Wire.write(regAddr);```. For nu at afslutte forespørgelsen og afsende data, kaldes funktionen ```Wire.endTransmission(devAddr);```. Hermed afsendes data til enheden. 

For nu at modtage data fra enheden, kaldes igen funktionen ```Wire.beginTransmission(devAddr);```. Herefter kaldes ```Wire.requestFrom(devAddr, size);```, hvor ```size``` definere antallet af bytes der ønskes modtaget. Antallet af bytes der er modtaget kan nu findes ved ```Wire.avaliable());``` og der kan læse en byte ved at bruge ```Wire.read();```.

Funktionen i ```I2Cdev``` biblioteket der læser bytes er som følger (fra ```I2Cdev.cpp``` filen):
```C++
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
	int8_t count = 0;
    uint32_t t1 = millis();
	for (uint8_t k = 0; k < length; k += min((int)length, BUFFER_LENGTH)) {
		Wire.beginTransmission(devAddr);
		Wire.write(regAddr);
		Wire.endTransmission();
		Wire.beginTransmission(devAddr);
		Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

		for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
			data[count] = Wire.read();
		}
	}
    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout
    return count;
}
```

Funktionen i ```I2Cdev``` biblioteket der skriver bytes er som følger (fra ```I2Cdev.cpp``` filen):
```C++
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
	uint8_t status = 0;
	Wire.beginTransmission(devAddr);
	Wire.write((uint8_t) regAddr); // send address
    for (uint8_t i = 0; i < length; i++) {
		Wire.write((uint8_t) data[i]);
    }
	Wire.endTransmission();
    return status == 0;
}
```

### MPU6050 biblioteket 
Da der nu er etableret I2C kommnukation med MPU6050, kan MPU6050 tilgåes via sine interne registre. Dette gøres ved at skrives eller læse bytes fra de interne registre i MPU6050. En liste over MPU6050's registre kan findes i dets datablad. 

Funktionen til at læse accleerometer- og gyroskop data gøres ved fælgende (fra ```MPU6050.cpp``` filen):
```C++
void MPU6050::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
```
Hvor ```MPU6050_RA_ACCEL_XOUT_H``` er addressen på x-aksen af accelerometerdata, defineret i ```MPU6050.h``` filen ved:
```C++
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
```