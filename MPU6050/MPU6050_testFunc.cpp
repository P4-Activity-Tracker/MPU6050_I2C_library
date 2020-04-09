

#include <MPU6050_testFunc.h>

void MPU6050testSampleTimeAccl(MPU6050 *imuInstance) {
	Serial.println("Performing sample time test on accl sample.")
	Serial.print("Preparing sample time test...");
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	uint8_t repetitions = 10;
	uint32_t timeDiffArray[repetitions];
	uint32_t startTime;
	uint32_t endTime;
	Serial.println("Ok");
	Serial.println("Starting sample time test...");
	for (uint8_t i = 0; i < repetitions; i++) {
		Serial.print("Test nr: ");
		Serial.print(i+1);
		Serial.print("... ");
		startTime = micros();
		imuInstance->getAcceleration(&ax, &ay, &az);
		endTime = micros();
		timeDiffArray[i] = endTime - startTime;
		Serial.print("Time differences was: ");
		Serial.print(timeDiffArray[i]);
		Serial.println(" microseconds. Ok.");
	}
	Serial.println("Calculating average time difference...");
	uint64_t sumDiffTime = 0;
	for (uint8_t i = 0; i < repetitions; i++) {
		sumDiffTime = sumDiffTime + timeDiffArray[i];
	}
	float avgDiffTime = (float)sumDiffTime/repetitions;
	float avgDiffTimeMs = avgDiffTime/1000;
	Serial.print("Average sample time was: ");
	Serial.print(avgDiffTime);
	Serial.print(" microseconds. Or ");
	Serial.print(avgDiffTimeMs);
	Serial.println(" milliseconds.");
}

void MPU6050testSampleTimeGyro(MPU6050 *imuInstance) {
	Serial.println("Performing sample time test on gyro sample.")
	Serial.print("Preparing sample time test...");
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	uint8_t repetitions = 10;
	uint32_t timeDiffArray[repetitions];
	uint32_t startTime;
	uint32_t endTime;
	Serial.println("Ok");
	Serial.println("Starting sample time test...");
	for (uint8_t i = 0; i < repetitions; i++) {
		Serial.print("Test nr: ");
		Serial.print(i+1);
		Serial.print("... ");
		startTime = micros();
		imuInstance.getRotation(&gx, &gy, &gz);
		endTime = micros();
		timeDiffArray[i] = endTime - startTime;
		Serial.print("Time differences was: ");
		Serial.print(timeDiffArray[i]);
		Serial.println(" microseconds. Ok.");
	}
	Serial.println("Calculating average time difference...");
	uint64_t sumDiffTime = 0;
	for (uint8_t i = 0; i < repetitions; i++) {
		sumDiffTime = sumDiffTime + timeDiffArray[i];
	}
	float avgDiffTime = (float)sumDiffTime/repetitions;
	float avgDiffTimeMs = avgDiffTime/1000;
	Serial.print("Average sample time was: ");
	Serial.print(avgDiffTime);
	Serial.print(" microseconds. Or ");
	Serial.print(avgDiffTimeMs);
	Serial.println(" milliseconds.");
}

void MPU6050testSampleTime6D(MPU6050 *imuInstance) {
	Serial.println("Performing sample time test on 6D sample.")
	Serial.print("Preparing sample time test...");
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	uint8_t repetitions = 10;
	uint32_t timeDiffArray[repetitions];
	uint32_t startTime;
	uint32_t endTime;
	Serial.println("Ok");
	Serial.println("Starting sample time test...");
	for (uint8_t i = 0; i < repetitions; i++) {
		Serial.print("Test nr: ");
		Serial.print(i+1);
		Serial.print("... ");
		startTime = micros();
		imuInstance->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		endTime = micros();
		timeDiffArray[i] = endTime - startTime;
		Serial.print("Time differences was: ");
		Serial.print(timeDiffArray[i]);
		Serial.println(" microseconds. Ok.");
	}
	Serial.println("Calculating average time difference...");
	uint64_t sumDiffTime = 0;
	for (uint8_t i = 0; i < repetitions; i++) {
		sumDiffTime = sumDiffTime + timeDiffArray[i];
	}
	float avgDiffTime = (float)sumDiffTime/repetitions;
	float avgDiffTimeMs = avgDiffTime/1000;
	Serial.print("Average sample time was: ");
	Serial.print(avgDiffTime);
	Serial.print(" microseconds. Or ");
	Serial.print(avgDiffTimeMs);
	Serial.println(" milliseconds.");
}