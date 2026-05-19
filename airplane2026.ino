#include "MPU6050.h"
#include "Wire.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <MadgwickAHRS.h>
#include <SparkFun_u-blox_GNSS_v3.h>

static constexpr int I2C_SDA = 32; //connect both MPU6050 and magnetometer to 32,33 pins
static constexpr int I2C_SCL = 33;
static constexpr int SD_SCK  = 18;
static constexpr int SD_MISO = 19;
static constexpr int SD_MOSI = 23;
static constexpr int SD_CS   = 5;
static constexpr int BTN_PIN = 13; //
static constexpr int MPU_INT = 4;
static constexpr int GPS_TX = 17;
static constexpr int GPS_RX = 16;
static constexpr int BUZZER_PIN = 2;
#define mySerial Serial2

//binary data to write on sd card directly
struct LogData {
    unsigned long timestamp_us; //4 bytes
    float pitch; //4 bytes
    float roll; //4 bytes
    float g_force; //4 bytes
    int32_t latitude; 
    int32_t longtitude; 
    int32_t altitude;          
    int32_t velocity;
    int32_t vertical_velocity;
};

MPU6050 accelgyro(0x68);
SPIClass customSPI(VSPI);
File LogFile;
Madgwick filter;
SFE_UBLOX_GNSS_SERIAL myGNSS;

int16_t ax, ay, az, gx, gy, gz; // accelerometer and gyroscope data
float offsetx = 0, offsety = 0, offsetz = 0; // setting both accelerometer and gyroscope starting offsets as zero to set later in calibration 
float gyro_offsetx = 0, gyro_offsety = 0, gyro_offsetz = 0; 
int32_t latitude = 0; // gnss
int32_t longtitude = 0; // gnss
int32_t altitude = 0; // ABOVE SEA LEVEL
int32_t velocity = 0;
int32_t vertical_velocity = 0;


volatile bool buttonPressed = false;
volatile bool mpuInterrupt = false;
static int counter = 0; // counter is used so data is flushed into the sd card only every 100 loops to reduce write time
bool sd_error_flag = false; //error flag for microsd card
unsigned long lastTime = 0;

void calibrate();
void write_logs(const float &pitch, const float &roll, const float &g_force, const int32_t &latitude, const int32_t &longtitude, const int32_t &altitude, const int32_t &velocity, const int32_t &vertical_velocity);
void IRAM_ATTR handleButtonInterrupt();
void IRAM_ATTR handleMPUInterrupt();
void beepSuccess();
void beepError();
void beepReady();
void runPreFlightSequence();


void setup(){
    Serial.begin(115200);
    mySerial.begin(115200, SERIAL_8N1, GPS_TX, GPS_RX);

    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_PIN), handleButtonInterrupt, FALLING);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // buzzer off

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 

    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(2);
    accelgyro.setFullScaleGyroRange(0);
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    accelgyro.setRate(9); 

    pinMode(MPU_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), handleMPUInterrupt, RISING);
    accelgyro.setIntDataReadyEnabled(true);

    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    customSPI.begin(SD_SCK, SD_MISO, SD_MOSI, -1);

    while (myGNSS.begin(mySerial) == false) {
        Serial.println(F("u-blox GNSS not detected. Retrying..."));
        delay(1000);
    }
    myGNSS.setUART1Output(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(10); 
    myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    myGNSS.setAutoPVT(true); 

    filter.begin(100);

    runPreFlightSequence(); 
}

void loop(){


    if (myGNSS.getPVT()) {
        if (myGNSS.getGnssFixOk()) {
            latitude = myGNSS.getLatitude();
            longtitude = myGNSS.getLongitude();
            altitude = myGNSS.getAltitudeMSL();
            velocity = myGNSS.getGroundSpeed();
            vertical_velocity = myGNSS.getNedDownVel();
        } 
    }

    if (mpuInterrupt){
        mpuInterrupt = false; 

        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0f; // convert microseconds to seconds
        lastTime = currentTime;

        if (dt > 0.0f) {
            float trueFrequency = 1.0f / dt;
            filter.begin(trueFrequency);
        }



        accelgyro.getIntStatus();
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        //accelerometer data to acceleration
        float aX = ((float)ax - offsetx)/4096.0;
        float aY = ((float)ay - offsety)/4096.0;
        float aZ = (((float)az - offsetz)/4096.0) + 1.0; 
        
        // gyro data to gyro rates (deg/s)
        float dX = ((float)gx - gyro_offsetx) / 131.0;
        float dY = ((float)gy - gyro_offsety) / 131.0;
        float dZ = ((float)gz - gyro_offsetz) / 131.0; 
        
        filter.updateIMU(dX, dY, dZ, aX, aY, aZ);

        float madgwick_pitch = filter.getPitch();
        float madgwick_roll = filter.getRoll();

        float g_force = aZ;

        // //for testing
        // Serial.print(madgwick_pitch);
        // Serial.print(",");
        // Serial.println(madgwick_roll);

        write_logs(madgwick_pitch, madgwick_roll, g_force, latitude, longtitude, altitude, velocity, vertical_velocity);
    }
}

void calibrate(){
    Serial.println("**Calibrating... DO NOT MOVE AIRPLANE**");
    accelgyro.setIntDataReadyEnabled(false);

    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;
    int samples = 500;
    unsigned long nextSampleTime = micros();
    
    for (int i = 0; i < samples; i++) {
        
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;
        
        nextSampleTime += 10000;
        while (micros() < nextSampleTime) {
            yield();
        }
    }

    offsetx = ((float)sumAx / samples);
    offsety = ((float)sumAy / samples);
    offsetz = ((float)sumAz / samples);

    gyro_offsetx = (float)sumGx / samples;
    gyro_offsety = (float)sumGy / samples;
    gyro_offsetz = (float)sumGz / samples;

    accelgyro.setIntDataReadyEnabled(true);
    Serial.println("**Calibration FINISHED**");
}

void write_logs(const float &pitch, const float &roll, const float &g_force, const int32_t &latitude, const int32_t &longtitude, const int32_t &altitude, const int32_t &velocity, const int32_t &vertical_velocity){

    //avoid writing to sd card if (OPEN file failed) OR if (sd error flag = 1 from previously)
    if(sd_error_flag || !LogFile) {
        return; 
    }

    LogData data;
    data.timestamp_us = micros();
    data.pitch = pitch;
    data.roll = roll;
    data.g_force = g_force;
    data.latitude = latitude;
    data.longtitude = longtitude; 
    data.altitude = altitude;
    data.velocity = velocity;
    data.vertical_velocity = vertical_velocity;

    // .write() function returns the size of the data written to the microsd card
    size_t bytesWritten = LogFile.write((const uint8_t *)&data, sizeof(LogData));


    // if (the return of .write() isn't equal to the actual size of the data) then (trigger sd error flag = 1)
    if (bytesWritten != sizeof(LogData)) {
        Serial.println("SD Card WRITE Failure. Halting logging.");
        LogFile.close();
        sd_error_flag = true;
        return;
    }

    counter++;

    if (counter >= 100) {
        LogFile.flush();
        counter = 0;
    }
    
}

void IRAM_ATTR handleButtonInterrupt(){
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    
    if (interruptTime - lastInterruptTime > 200) {
        buttonPressed = true;
    }
    lastInterruptTime = interruptTime;
}

void IRAM_ATTR handleMPUInterrupt(){
    mpuInterrupt = true;
}

void beepSuccess() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200); // one quick, happy beep
    digitalWrite(BUZZER_PIN, LOW);
}

void beepError() {
    for(int i = 0; i < 15; i++) { // fast beep for ~3 seconds
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}

void beepReady() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(2000); // 2-second continuous tone
    digitalWrite(BUZZER_PIN, LOW);
}

void runPreFlightSequence() {
    Serial.println("System powered on. Waiting for button press...");
    
    // 1. wait for button press (input_pullup)
    while(digitalRead(BTN_PIN) == HIGH) {
        delay(50);
    }
    
    Serial.println("Button pressed! Checking SD Card...");

    // 2. check sd card
    if (!SD.begin(SD_CS, customSPI, 4000000)) {
        Serial.println("SD Card MOUNT failed.");
        sd_error_flag = true;
        beepError();
        while(true) { delay(1000); } // HALT
    } else {
        char filename[32];
        bool fileOpened = false;
        
        for (int i = 0; i < 1000; i++) {
            sprintf(filename, "/flight_%03d.bin", i); 
            
            if (!SD.exists(filename)) {
                LogFile = SD.open(filename, FILE_WRITE);
                if (LogFile) {
                    Serial.print("Created new log: ");
                    Serial.println(filename);
                    fileOpened = true;
                }
                break; 
            }
        }

        if (!fileOpened) {
            Serial.println("Failed to OPEN new file.");
            sd_error_flag = true;
            beepError();
            while(true) { delay(1000); } // HALT
        } else {
            beepSuccess(); 
        }
    }

    // 3. calibrate MPU
    calibrate(); 

    // 4. wait for GPS lock
    Serial.println("Waiting for 3D GPS Lock...");
    while (true) {
        if (myGNSS.getPVT() && myGNSS.getGnssFixOk()) {
            break; // LOCK!
        }
        delay(100); 
    }

    // 5. FLIGHT READY!
    Serial.println("GPS Locked, calibrated, SD mount ready. FLIGHT READY!");
    beepReady();

    accelgyro.getIntStatus();
    mpuInterrupt = false;
    lastTime = micros();
}