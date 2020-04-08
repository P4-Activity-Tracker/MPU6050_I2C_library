# MPU6050_I2C_library
Library for sampling using the MPU6050 IMU

## Including the MPU6050 library in PlatformIO
Place the two folders ```I2Cdev``` and ```MPU6050``` and place them in the ```lib``` created by PlatformIO. Add the following to the top of the ```.cpp``` file:
```C
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
```
The MPU6050 library is now ready to use.

## Using the MPU6050 library
To use the MPU6050 library, first initialize the Wire library by adding the following to setup:
```C++
void setup() {
	...
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	...
}
```
Now initialize the MPU6050 class in the top of the file and prepare variables for the accelerometer and gyroscope data:
```C++
MPU6050 accelgyro; // Initialize MPU6050 class.
int16_t ax, ay, az; // Accelerometer data variables
int16_t gx, gy, gz; // Gyroscope data variables
...
void setup() {
	...
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	...
}
```
Now initialize the MPU6050 by adding the following to setup:
```C++
MPU6050 imu; // Initialize MPU6050 class.
int16_t ax, ay, az; // Accelerometer data variables
int16_t gx, gy, gz; // Gyroscope data variables
...
void setup() {
	...
	Wire.begin(SDA_pin, SCL_pin); // Where SDA is the pin number connected to SDA
	imu.initialize(); // Initialize the MPU6050
	...
}
```
Now to get accelerometer and gyroscope data, simply run: 
```C++
imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Get accelerometer and gyroscope data
```
## Usefull functions
To test connection to the MPU6050:
```C++
imu.testConnection() ; // Test MPU6050 connection
```
Set or get sampling rate:
```C++
uint8_t rate = imu.getRate();
imu.setRate(rate); // Takes a uint8_t as input
```





