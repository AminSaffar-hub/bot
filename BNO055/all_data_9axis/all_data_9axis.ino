#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
float yaw;
float Xm;
float Ym;
float heading ;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
    delay(500);
}

/*Display sensor calibration status*/

void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    //Serial.print("\t");
    if (!system)
    {
        //Serial.print("! ");
    }

    /* Display the individual values */
//    Serial.print("Sys:");
//    Serial.print(system, DEC);
//    Serial.print(" G:");
//    Serial.print(gyro, DEC);
//    Serial.print(" A:");
//    Serial.print(accel, DEC);
//    Serial.print(" M:");
//    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void)
{
    Serial.begin(115200);
    delay(1000);
    //Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        //Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
       // Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        //Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        //Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(500);

    /* Display some basic information on this sensor */
    //displaySensorDetails();

    /* Optional: Display current status */
    //displaySensorStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib){
        //Serial.println(" sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
      //  Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    //Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    //Serial.println("Data stored to EEPROM.");
    displayCalStatus();
    delay(500);
}
void loop() {
    sensors_event_t event;
    bno.getEvent(&event);

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro= bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> VECTOR_LINEARACCEL = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

/* affichage data accelerometer */
    //Serial.print("accX: ");
    //Serial.print(",");
    Serial.print(acc.x());
    //Serial.print("accY: ");
    Serial.print(",");
    Serial.print(acc.y());
    //Serial.print("accZ: ");
    Serial.print(",");
    Serial.print(acc.z());
    //Serial.print("\t");
/* affichage data gyro */
    //Serial.print(" gyroX: ");
    Serial.print(",");
    Serial.print(gyro.x());
    //Serial.print(" gyroY: ");
    Serial.print(",");
    Serial.print(gyro.y());
    //Serial.print(" gyroZ: ");
    Serial.print(",");
    Serial.print(gyro.z());
    //Serial.print("\t");
/* affichage data magnetometer */
    //Serial.print(" magX: ");
    Serial.print(",");
    Serial.print(mag.x());
    //Serial.print(" magY: ");
    Serial.print(",");
    Serial.print(mag.y());
    //Serial.print(" magZ: ");
    Serial.print(",");
    Serial.print(mag.z());
    //Serial.print("\t");
//quaternion data
    imu::Quaternion quat = bno.getQuat();
    //Serial.print(" qW: ");
    Serial.print(",");
    Serial.print(quat.w(), 4);
    //Serial.print(" qX: ");
    Serial.print(",");
    Serial.print(quat.x(), 4);
    //Serial.print(" qY: ");
    Serial.print(",");
    Serial.print(quat.y(), 4);
    //Serial.print(" qZ: ");
    Serial.print(",");
    Serial.print(quat.z(), 4);
    Serial.println("");
    
    

//     Xm= mag.x();
//   Ym= mag.y();
//    heading= atan2(Ym,Xm)/(2*3.14)*360;//retour en radian 
//    yaw = map(heading, -180, 180, 0, 360);

//    Serial.println(yaw);
    delay(BNO055_SAMPLERATE_DELAY_MS);
}
