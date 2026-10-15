#include <bluefruit.h>
#include "Adafruit_SPIFlash.h"
#include <InternalFileSystem.h>
#include "LSM6DS3.h" // Assuming Seeed Studio LSM6DS3 library or compatible
#include "Wire.h"
#include <PDM.h> //pdm library from adafruit?
#include <opus.h> //arduino-libopus
#include "MadgwickAHRS.h"
Adafruit_FlashTransport_QSPI flashTransport;
// microphone and encoding 
OpusEncoder *encoder;
int opus_error;
#define SAMPLE_RATE 16000   // this is consistent with what opus and the pdm library support
#define CHANNELS 1 // the microphone is mono
#define APPLICATION OPUS_APPLICATION_RESTRICTED_LOWDELAY // this seems to be the lightest weight APPLICATION setting (VOIP is slow...)
#define BITRATE 10000       // this number is flexible.  Higher would reduce complexity but increase the encoding size meaning increased communication delay
#define FRAME_SIZE 160      // 10ms frame time, which also sets the sensor streaming rate to 100hz
#define BUFF_SIZE 16000 // we have a 1s buffer in case of delays streaming data (will overwrite and cause problems otherwise)
#define COMPLEXITY 0
short pdmBuffer[FRAME_SIZE]; // holds data read in the pdm interrupt, which is immediately written to the sample buffer
short sampleBuffer[BUFF_SIZE]; // this is the go-between for pdm and encoding
volatile int writeIndex = 0; //where data will be written into the sample buffer
volatile long samplesWritten = 0; //how many total samples have been written so far 
int readIndex = 0; //where data will be read from the sample buffer
long samplesRead = 0; //how many samples have been read so far
short encodeBuffer[FRAME_SIZE]; // used to hold the data being processed by the encoder
unsigned char opus_output[FRAME_SIZE]; // an oversized buffer for opus (realistically, it'll be about 10-20 bytes)

const int BATTERY_READ_EN = 14; //this works, it might be D13, but The D and # are not necessarily consistent.
const int CHARGE_PIN = 22; //used to enable 100mA charge current
const int VIBRATION_PIN = D0;
const int BUTTON_PIN = D10;

//we have 2 capacitive sensors, each with a drive and read pin
const int CAP_DRIVE1_PIN = D4;
const int CAP_SENSE1_PIN = D9;
const int CAP_DRIVE2_PIN = D6;
const int CAP_SENSE2_PIN = D7;
const int CAP_READ_TIMEOUT_COUNT = 50; //max loop iterations to wait for the pin to go high

//IMU FIFO draining.  One sample set is 6 words (gx,gy,gz,ax,ay,az) = 12 bytes.
#define FIFO_WORDS_PER_SET 6
#define MAX_SETS_PER_PACKET 16   //bound the work per packet so a backlog can't stall the loop
#define FIFO_WAIT_TIMEOUT_US 20000 //never spin forever waiting on the watermark

LSM6DS3 myIMU(I2C_MODE, 0x6A); // Default I2C address is 0x6A

// the gyro and accell functions return a float, so we convert them to their target ranges back to an int_16 using these scale factors
const float ACCEL_SCALE_FACTOR = 32767.0f / 16.0f;  // Max int16_t / G-range (accelRange is 16g;
                                                    // at /4.0 anything past 4g clipped.  16g is also
                                                    // the sensor's native LSB here, so nothing is lost)
const float GYRO_RANGE_DPS = 2000.0f; // Must match myIMU.settings.gyroRange below.
const float GYRO_SCALE_FACTOR = 32767.0f / GYRO_RANGE_DPS;

// BEGIN AUTO ACCEL CALIBRATION
// corrected_g = (measured_g - bias_g) / gain.  The GUI updates only these lines
// after it validates three complete six-position calibration trials.
float ACCEL_BIAS_G[3] = {-0.011650f, 0.000900f, 0.043467f};
float ACCEL_GAIN[3] = {1.006283f, 1.007433f, 1.009633f};
// END AUTO ACCEL CALIBRATION

// BEGIN AUTO GYRO CALIBRATION
// corrected_dps = (measured_dps - bias_dps) / gain.  The GUI replaces this
// block after three complete six-position-plus-rotation calibration trials.
float GYRO_BIAS_DPS[3] = {0.000000f, 0.000000f, 0.000000f};
float GYRO_GAIN[3] = {1.000000f, 1.000000f, 1.000000f};
// END AUTO GYRO CALIBRATION

// A small versioned record in the nRF52's internal LittleFS. It is separate
// from program flash, so calibration can be replaced over BLE and survives
// power cycles and firmware restarts.
const char* CALIBRATION_FILE = "/vring_calibration.bin";
const uint32_t CALIBRATION_MAGIC = 0x31435256UL; // "VRC1" in little-endian bytes
const uint16_t CALIBRATION_VERSION = 1;
bool calibrationFilesystemReady = false;
struct __attribute__((packed)) CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float accelBias[3];
  float accelGain[3];
  float gyroBias[3];
  float gyroGain[3];
  uint32_t checksum;
};

uint32_t calibrationChecksum(const uint8_t* bytes, size_t length) {
  uint32_t hash = 2166136261UL; // FNV-1a, shared with the desktop GUI.
  for (size_t i = 0; i < length; i++) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool calibrationValuesAreValid(const CalibrationRecord& record) {
  if (record.magic != CALIBRATION_MAGIC || record.version != CALIBRATION_VERSION) return false;
  if (record.checksum != calibrationChecksum((const uint8_t*)&record, sizeof(record) - sizeof(record.checksum))) return false;
  for (int i = 0; i < 3; i++) {
    if (!isfinite(record.accelBias[i]) || !isfinite(record.gyroBias[i]) ||
        !isfinite(record.accelGain[i]) || !isfinite(record.gyroGain[i])) return false;
    if (fabs(record.accelBias[i]) > 0.30f || fabs(record.gyroBias[i]) > 20.0f ||
        record.accelGain[i] < 0.50f || record.accelGain[i] > 1.50f ||
        record.gyroGain[i] < 0.50f || record.gyroGain[i] > 1.50f) return false;
  }
  return true;
}

void installCalibration(const CalibrationRecord& record) {
  memcpy(ACCEL_BIAS_G, record.accelBias, sizeof(ACCEL_BIAS_G));
  memcpy(ACCEL_GAIN, record.accelGain, sizeof(ACCEL_GAIN));
  memcpy(GYRO_BIAS_DPS, record.gyroBias, sizeof(GYRO_BIAS_DPS));
  memcpy(GYRO_GAIN, record.gyroGain, sizeof(GYRO_GAIN));
}

CalibrationRecord currentCalibrationRecord(void) {
  CalibrationRecord record = {CALIBRATION_MAGIC, CALIBRATION_VERSION, 0, {}, {}, {}, {}, 0};
  memcpy(record.accelBias, ACCEL_BIAS_G, sizeof(record.accelBias));
  memcpy(record.accelGain, ACCEL_GAIN, sizeof(record.accelGain));
  memcpy(record.gyroBias, GYRO_BIAS_DPS, sizeof(record.gyroBias));
  memcpy(record.gyroGain, GYRO_GAIN, sizeof(record.gyroGain));
  record.checksum = calibrationChecksum((const uint8_t*)&record, sizeof(record) - sizeof(record.checksum));
  return record;
}

void loadStoredCalibration(void) {
  calibrationFilesystemReady = InternalFS.begin();
  if (!calibrationFilesystemReady) return;
  Adafruit_LittleFS_Namespace::File file = InternalFS.open(CALIBRATION_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ);
  CalibrationRecord record;
  bool valid = file && file.read(&record, sizeof(record)) == (int)sizeof(record);
  if (file) file.close();
  if (valid && calibrationValuesAreValid(record)) installCalibration(record);
}

bool saveStoredCalibration(const CalibrationRecord& record) {
  if (!calibrationValuesAreValid(record)) return false;
  if (!calibrationFilesystemReady) {
    calibrationFilesystemReady = InternalFS.begin();
    if (!calibrationFilesystemReady) return false;
  }
  InternalFS.remove(CALIBRATION_FILE);
  Adafruit_LittleFS_Namespace::File file = InternalFS.open(CALIBRATION_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (!file) return false;
  bool saved = file.write((const uint8_t*)&record, sizeof(record)) == sizeof(record);
  file.flush();
  file.close();
  if (saved) installCalibration(record);
  return saved;
}

// BLE Service & Characteristics
BLEService        vringService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEDis            bledis;
//accel(6), quaternion(6), capsense(4), battery(2), temperature(2), sample counter(2), gyro(6)
//Gyro is appended so the original 22-byte field offsets remain unchanged.
#define SENSOR_PACKET_LEN 28
BLECharacteristic sensorDataCharacteristic("19B10002-E8F2-537E-4F6C-D104768A121A", BLERead | BLENotify, SENSOR_PACKET_LEN);
BLECharacteristic vibrationControlCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1219", BLEWrite, 1); //0-255 power
BLECharacteristic calibrationCharacteristic("19B10007-E8F2-537E-4F6C-D104768A1222", BLERead | BLEWrite, sizeof(CalibrationRecord));
BLECharacteristic opusDataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1220", BLERead | BLENotify, FRAME_SIZE); //streaming compressed opus data
BLECharacteristic buttonInputCharacteristic("19B10006-E8F2-537E-4F6C-D104768A1221", BLERead | BLENotify, 1);
Madgwick * m = NULL;
float gx,gy,gz,ax,ay,az;
//Counts every IMU sample set pulled from the FIFO, including ones we drain but do not
//transmit.  The FIFO runs at a fixed 400Hz, so sample time is counter * 2.5ms exactly -
//tied to the sensor's own clock rather than to when the MCU got around to reading it.
//A gap in this counter is the receiver's only way to see dropped or skipped samples.
uint16_t sampleCounter = 0;
float qw,qx,qy,qz;
uint8_t lastButtonState = HIGH;
void QSPIF_sleep(void)
{
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  flashTransport.end();  
}


void enableIMU(bool enable){
  
  if(enable){
    pinMode(PIN_LSM6DS3TR_C_POWER,OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER,HIGH); 
    delay(1); //make sure IMU is on
    myIMU.begin();
  }else{
    pinMode(PIN_LSM6DS3TR_C_POWER,OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER,LOW); //shut down IMU
  }
}
void setup() {

  enableIMU(false);
  encoder = opus_encoder_create(SAMPLE_RATE, CHANNELS, APPLICATION, &opus_error);
  opus_encoder_ctl(encoder, OPUS_SET_BITRATE(BITRATE));
  opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(COMPLEXITY)); // Complexity 0-10, 2 is a good balance
  QSPIF_sleep();
  loadStoredCalibration();
  //initialize pins and modes
  // pinMode(LED_RED,OUTPUT);digitalWrite(LED_RED,HIGH);
  // pinMode(LED_GREEN,OUTPUT);digitalWrite(LED_GREEN,HIGH);
  // pinMode(LED_BLUE,OUTPUT);digitalWrite(LED_BLUE,HIGH); //all color pins off
   pinMode(VIBRATION_PIN, OUTPUT); //turn off vibration
    digitalWrite(VIBRATION_PIN, LOW);
   pinMode(BUTTON_PIN, INPUT_PULLUP);
   lastButtonState = digitalRead(BUTTON_PIN) == LOW ? 1 : 0;
   pinMode(CAP_DRIVE1_PIN, OUTPUT);
  digitalWrite(CAP_DRIVE1_PIN, LOW);
  pinMode(CAP_SENSE1_PIN, INPUT);
  pinMode(CAP_DRIVE2_PIN, OUTPUT);
  digitalWrite(CAP_DRIVE2_PIN, LOW);
  pinMode(CAP_SENSE2_PIN, INPUT);
  pinMode(CHARGE_PIN,OUTPUT);
  digitalWrite(CHARGE_PIN,LOW);

  pinMode(BATTERY_READ_EN, OUTPUT); 
  digitalWrite(BATTERY_READ_EN, HIGH); //disable battery reading
  analogReference(AR_INTERNAL_2_4); //2.4V analog reference for battery reading

  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(FRAME_SIZE * 2); 
  PDM.end();
  //not sure what of this is important to increase bandwidth, but trying everything
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.autoConnLed(false); // Disable default connection LED
  Bluefruit.setTxPower(0);      // Max power
  Bluefruit.Periph.setConnInterval(6, 8); // 7.5ms to 10ms connection interval

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Bluefruit.setName("VRING");
  bledis.setManufacturer("VEL");
  bledis.setModel("VRING2"); // Model name reflecting sensor suite
  bledis.begin();

  vringService.begin();

  sensorDataCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  sensorDataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensorDataCharacteristic.setFixedLen(SENSOR_PACKET_LEN); // IMU + Cap + Battery + Temp + counter
  sensorDataCharacteristic.begin();
  uint8_t initialSensorData[SENSOR_PACKET_LEN] = {0}; // Initialize with zeros
  sensorDataCharacteristic.write(initialSensorData, sizeof(initialSensorData));

  vibrationControlCharacteristic.setProperties(CHR_PROPS_WRITE);
  vibrationControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  vibrationControlCharacteristic.setFixedLen(1);
  vibrationControlCharacteristic.setWriteCallback(vibration_write_callback);
  vibrationControlCharacteristic.begin();
  vibrationControlCharacteristic.write8(0x00); // Initial vibration off

  calibrationCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  calibrationCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  calibrationCharacteristic.setFixedLen(sizeof(CalibrationRecord));
  calibrationCharacteristic.setWriteCallback(calibration_write_callback);
  calibrationCharacteristic.begin();
  CalibrationRecord initialCalibration = currentCalibrationRecord();
  calibrationCharacteristic.write((const uint8_t*)&initialCalibration, sizeof(initialCalibration));

  opusDataCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  opusDataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  opusDataCharacteristic.setMaxLen(FRAME_SIZE); 
  opusDataCharacteristic.begin();

  buttonInputCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  buttonInputCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  buttonInputCharacteristic.setFixedLen(1);
  buttonInputCharacteristic.begin();
  buttonInputCharacteristic.write8(lastButtonState);

  startAdv();
  Serial.begin(230400);
}

int readCapacitiveValue(int drivePin, int sensePin, int maxCount) {
    pinMode(sensePin, INPUT);
    digitalWrite(drivePin, HIGH); // Start charging the sense pin through body capacitance
    int count = 0;
    while (!digitalRead(sensePin) && count < maxCount) { // Count how long it takes to charge
        count++;
    }
    // Reset pins for next read or other uses
    digitalWrite(drivePin, LOW);
    pinMode(sensePin, OUTPUT); 
    digitalWrite(sensePin, LOW);
    return count;
}

//Pull one word out of the IMU FIFO in a single I2C transaction.
//The LSM6DS3 auto-increment does NOT wrap between FIFO_DATA_OUT_L (0x3E) and _H (0x3F)
//- 0x40 is TIMESTAMP0_REG - so reading more than 2 bytes at a time returns one real
//FIFO word followed by unrelated registers.  2 bytes is the largest safe read, and it
//still halves the cost of fifoRead(), which spends a transaction per byte.
bool fifoReadWord(int16_t &out) {
  uint8_t b[2];
  if (myIMU.readRegisterRegion(b, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L, 2) != IMU_SUCCESS) return false;
  out = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  return true;
}

// Read the accelerometer's dedicated output registers as one six-byte transfer.
// Unlike FIFO_DATA_OUT, these registers are always X, Y, Z in that order.  The
// calibration packet must use this path so a FIFO recovery cannot swap its axes.
bool readDirectAccel(void) {
  const uint8_t ACCEL_OUTX_L = 0x28;
  uint8_t b[6];
  if (myIMU.readRegisterRegion(b, ACCEL_OUTX_L, sizeof(b)) != IMU_SUCCESS) return false;
  const int16_t rawX = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  const int16_t rawY = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  const int16_t rawZ = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  ax = myIMU.calcAccel(rawX);
  ay = myIMU.calcAccel(rawY);
  az = myIMU.calcAccel(rawZ);
  ax = (ax - ACCEL_BIAS_G[0]) / ACCEL_GAIN[0];
  ay = (ay - ACCEL_BIAS_G[1]) / ACCEL_GAIN[1];
  az = (az - ACCEL_BIAS_G[2]) / ACCEL_GAIN[2];
  return true;
}

// The gyro uses the same fixed-order direct-register path as acceleration.  This
// keeps stationary bias logging independent of the FIFO's mixed sensor pattern.
bool readDirectGyro(void) {
  const uint8_t GYRO_OUTX_L = 0x22;
  uint8_t b[6];
  if (myIMU.readRegisterRegion(b, GYRO_OUTX_L, sizeof(b)) != IMU_SUCCESS) return false;
  const int16_t rawX = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  const int16_t rawY = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  const int16_t rawZ = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  gx = myIMU.calcGyro(rawX);
  gy = myIMU.calcGyro(rawY);
  gz = myIMU.calcGyro(rawZ);
  gx = (gx - GYRO_BIAS_DPS[0]) / GYRO_GAIN[0];
  gy = (gy - GYRO_BIAS_DPS[1]) / GYRO_GAIN[1];
  gz = (gz - GYRO_BIAS_DPS[2]) / GYRO_GAIN[2];
  return true;
}

//Where the next word sits within the 6-word gx,gy,gz,ax,ay,az pattern.
bool fifoGetPattern(uint16_t &pattern) {
  uint8_t lo = 0, hi = 0;
  if (myIMU.readRegister(&lo, LSM6DS3_ACC_GYRO_FIFO_STATUS3) != IMU_SUCCESS) return false;
  if (myIMU.readRegister(&hi, LSM6DS3_ACC_GYRO_FIFO_STATUS4) != IMU_SUCCESS) return false;
  pattern = ((uint16_t)(hi & 0x03) << 8) | lo;
  return true;
}

// Align the FIFO read pointer with gyro X, the first word of one gx,gy,gz,ax,ay,az set.
// Return false rather than guessing if I2C failed mid-resynchronization.
bool fifoAlignToSetStart(void) {
  uint16_t pattern;
  if (!fifoGetPattern(pattern)) return false;

  const uint8_t nextWord = pattern % FIFO_WORDS_PER_SET;
  const uint8_t wordsToStart = (FIFO_WORDS_PER_SET - nextWord) % FIFO_WORDS_PER_SET;
  int16_t discard;
  for (uint8_t i = 0; i < wordsToStart; i++) {
    if (!fifoReadWord(discard)) return false;
  }

  // Do not perform a second pattern read here.  On this board the status register
  // can advance while the FIFO is being refilled, even though the read pointer is
  // correctly aligned after the discards above.
  return true;
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(vringService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(1000, 2000); // Advertising interval: 20ms to 152.5ms
  Bluefruit.Advertising.setFastTimeout(30);  // Fast advertising for 30 seconds
  Bluefruit.Advertising.start(0);            // Start advertising, 0 = continuous
}

void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  if (connection) {
    connection->requestPHY(); // Request 2M PHY for higher throughput if central supports it
    connection->requestDataLengthUpdate(); // Request larger Data Length Extension (DLE)
    connection->requestMtuExchange(247);   // Request larger MTU (ATT Maximum Transmission Unit)
    connection->requestConnectionParameter(6); // Request 7.5ms connection interval
    pinMode(VIBRATION_PIN,OUTPUT);
    digitalWrite(VIBRATION_PIN,LOW); 
    //analogWrite(VIBRATION_PIN,0);
    PDM.begin(1, SAMPLE_RATE);
    digitalWrite(BATTERY_READ_EN, LOW); //enable battery reading
    
    m = new Madgwick();
    m->begin(400);
    //Over-ride default settings if desired
    myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
    myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    myIMU.settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
    myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
    myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

    myIMU.settings.accelEnabled = 1;
    myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16    
    myIMU.settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
    myIMU.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
    myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
    myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
    myIMU.settings.tempEnabled = 1;
    
    //Non-basic mode settings
    myIMU.settings.commMode = 1;

    //FIFO control settings
    myIMU.settings.fifoThreshold = 6;  //one complete sample must be there
    myIMU.settings.fifoSampleRate = 400;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
    myIMU.settings.fifoModeWord = 6;  //FIFO mode.
    enableIMU(true);
    Wire1.setClock(400000UL);

    myIMU.fifoBegin();

    //fifoBegin() hard-codes FIFO_CTRL4 to 0x09, which enables the 3rd and 4th FIFO
    //datasets (sensor hub / step counter) at no decimation.  Nothing is wired to either,
    //so they contribute 6 zero words per cycle - making the FIFO pattern 12 words rather
    //than the 6 this sketch reads, and burning half the FIFO bandwidth on nothing.
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, 0x00);

    myIMU.fifoClear();
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  vibrationControlCharacteristic.write8(0x00); // Ensure vibration is off
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN,0);
  //analogWrite(VIBRATION_PIN, 0); //turn off any vibration
  PDM.end();
  digitalWrite(BATTERY_READ_EN, HIGH);  //disable battery reading
  //start again on the write/read
  writeIndex = 0; 
  samplesWritten = 0; 
  readIndex = 0; 
  samplesRead = 0; 
  enableIMU(false);

  NVIC_SystemReset(); //can't figure out how to go low power, so this is a kludge
}

void vibration_write_callback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if(data[0] > 0){
    pinMode(VIBRATION_PIN,OUTPUT);
    analogWrite(VIBRATION_PIN, data[0]);
  }else{
    pinMode(VIBRATION_PIN,OUTPUT);
    analogWrite(VIBRATION_PIN,LOW);
  }
    
}

float GetBatteryVoltage() {
  uint32_t adcCount = analogRead(PIN_VBAT); // defined by xiao
  float voltageAtADC = (adcCount / 1023.0f) * 2.4f;
  float batteryVoltage = voltageAtADC * (1510.0f / 510.0f); 
  return batteryVoltage;
}

void sendSensorPacket(){
  uint8_t sensorPayload[SENSOR_PACKET_LEN]; // 12 IMU + 4 Cap + 2 Battery + 2 Temp + 2 counter

  float s = qw < 0?-1:1;
  int16_t iqx = qx*32767*s; //flipped if w is negative so you ensure w > 1, which let's you solve for w = sqrt(1-x^2-y^2-z^2) 
  int16_t iqy = qy*32767*s;
  int16_t iqz = qz*32767*s; 

  int16_t accelX = (int16_t)(ax * ACCEL_SCALE_FACTOR);
  int16_t accelY = (int16_t)(ay * ACCEL_SCALE_FACTOR);
  int16_t accelZ = (int16_t)(az * ACCEL_SCALE_FACTOR);

  memcpy(sensorPayload + 0,  &accelX, sizeof(int16_t));
  memcpy(sensorPayload + 2,  &accelY, sizeof(int16_t));
  memcpy(sensorPayload + 4,  &accelZ, sizeof(int16_t));
  memcpy(sensorPayload + 6,  &iqx,  sizeof(int16_t));
  memcpy(sensorPayload + 8,  &iqy,  sizeof(int16_t));
  memcpy(sensorPayload + 10, &iqz,  sizeof(int16_t));

  // 2. Capacitive Sensor Data (next 4 bytes)
  int capVal1_raw = readCapacitiveValue(CAP_DRIVE1_PIN, CAP_SENSE1_PIN, CAP_READ_TIMEOUT_COUNT);
  int capVal2_raw = readCapacitiveValue(CAP_DRIVE2_PIN, CAP_SENSE2_PIN, CAP_READ_TIMEOUT_COUNT);

  int16_t capVal1_16bit = (int16_t)capVal1_raw;
  int16_t capVal2_16bit = (int16_t)capVal2_raw;

  memcpy(sensorPayload + 12, &capVal1_16bit, sizeof(int16_t));
  memcpy(sensorPayload + 14, &capVal2_16bit, sizeof(int16_t));

  // 3. Battery Voltage Data (next 2 bytes)
  float voltage_V = GetBatteryVoltage();
  uint16_t voltage_mV = (uint16_t)(voltage_V * 1000.0f);
  memcpy(sensorPayload + 16, &voltage_mV, sizeof(uint16_t));

  // 4. Temperature Data (last 2 bytes, Fahrenheit * 100)
  float tempF = myIMU.readTempF(); // Read temperature in Fahrenheit
  int16_t tempF_scaled = (int16_t)(tempF * 100.0f); // Scale to preserve two decimal places
  memcpy(sensorPayload + 18, &tempF_scaled, sizeof(int16_t));

  // 5. Sample counter (last 2 bytes).  Sample time = counter * 2.5ms; wraps every ~164s.
  memcpy(sensorPayload + 20, &sampleCounter, sizeof(uint16_t));

  // 6. Direct gyro data (degrees/sec), appended for stationary-bias collection.
  const int16_t gyroX = (int16_t)(gx * GYRO_SCALE_FACTOR);
  const int16_t gyroY = (int16_t)(gy * GYRO_SCALE_FACTOR);
  const int16_t gyroZ = (int16_t)(gz * GYRO_SCALE_FACTOR);
  memcpy(sensorPayload + 22, &gyroX, sizeof(int16_t));
  memcpy(sensorPayload + 24, &gyroY, sizeof(int16_t));
  memcpy(sensorPayload + 26, &gyroZ, sizeof(int16_t));

  // Notify the combined sensor data
  sensorDataCharacteristic.notify(sensorPayload, sizeof(sensorPayload));
}

void loop() {
  if (Bluefruit.connected()) { //we only send sensor data when connected
    // Sensor data is sent when opus audio data is ready and encoded

    //handle imu integration
    //Wait for the watermark, but bounded: a wedged IMU should show up as a stall we
    //can see rather than silently pacing the entire loop.
    unsigned long waitStart = micros();
    uint16_t fifoStatus = myIMU.fifoGetStatus();
    while( (fifoStatus & 0x8000) == 0 ) {
      if (micros() - waitStart > FIFO_WAIT_TIMEOUT_US) break;
      fifoStatus = myIMU.fifoGetStatus();
    }

    // FIFO overrun invalidates the unread-word count and can start reading halfway
    // through a pattern. Flush it; never reinterpret stale/misaligned words as axes.
    if (fifoStatus & 0x4000) {
      myIMU.fifoClear();
      return;
    }

    // A timeout means there is not yet a complete group.  Holding the prior display
    // value is better than notifying it again as a new sample.
    if ((fifoStatus & 0x8000) == 0) return;

    if (!fifoAlignToSetStart()) return;

    //Read the backlog once, then drain whole sets.  Polling the status register on every
    //iteration and reading each word a byte at a time cost 14 I2C transactions per set,
    //barely faster than the 400Hz fill rate - lose that race once and the FIFO never
    //empties, so the old drain-until-empty loop only exited on overrun (~0.5Hz).
    fifoStatus = myIMU.fifoGetStatus();
    if (fifoStatus & 0x4000) {
      myIMU.fifoClear();
      return;
    }
    uint16_t availableSets = (fifoStatus & 0x0FFF) / FIFO_WORDS_PER_SET;
    if (availableSets > MAX_SETS_PER_PACKET) availableSets = MAX_SETS_PER_PACKET;

    bool receivedFreshSet = false;
    uint16_t drainedSets = 0;
    for (uint16_t s = 0; s < availableSets; s++) {
      int16_t word[FIFO_WORDS_PER_SET];
      bool ok = true;
      for (int i = 0; i < FIFO_WORDS_PER_SET && ok; i++) ok = fifoReadWord(word[i]);
      if (!ok) break;

      //Count every set, not just transmitted ones, so the receiver can tell exactly how much
      //time each packet advanced and spot anything it missed.
      sampleCounter++;
      // The FIFO supplies the precise sample-clock advance, but its word-order
      // recovery must not feed the orientation filter.  The direct output
      // registers below are the fixed X/Y/Z, calibrated sensor values.
      receivedFreshSet = true;
      drainedSets++;
    }

    // Do not label previous accelerometer values as a new BLE measurement.
    if (!receivedFreshSet) return;

    // Update the filter once per transmitted packet using the same direct,
    // calibrated axes sent over BLE.  Its timestep is the number of ring
    // samples just drained, not the variable BLE/CPU scheduling interval.
    if (!readDirectAccel() || !readDirectGyro()) return;
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return;
    m->begin(400.0f / (float)drainedSets);
    m->updateIMU(gx,gy,gz,ax,ay,az);

    // Once per packet rather than once per sample - getQuaternion recomputes euler angles.
    //NB: the signature is getQuaternion(x,y,z,w) - w is LAST.  Passing (qw,qx,qy,qz) put
    //the real part into qz and shifted everything else, so the packet carried (q2,q3,q0).
    m->getQuaternion(qx,qy,qz,qw);
    if (isnan(qw) || isnan(qx) || isnan(qy) || isnan(qz)) { //safety net: restart a poisoned filter
      delete m;
      m = new Madgwick();
      m->begin(400);
      qw = qx = qy = qz = 0;
    }

    sendSensorPacket();

    uint8_t buttonState = digitalRead(BUTTON_PIN) == LOW ? 1 : 0;
    if (buttonState != lastButtonState) {
      lastButtonState = buttonState;
      buttonInputCharacteristic.write8(buttonState);
      buttonInputCharacteristic.notify(&buttonState, sizeof(buttonState));
    }
    // if(samplesWritten - samplesRead >= FRAME_SIZE){ //and when we have enough data to do a full microphone encoding

      

    //   //this creates a complete encode buffer.  Necessary because the read buffer could wrap around (should be quick enough)
    //   for(int i=0; i<FRAME_SIZE; i++){
    //     encodeBuffer[i] = sampleBuffer[readIndex];
    //     readIndex = (readIndex + 1) % BUFF_SIZE;
    //   }
    //   //this encodes the data (most expensive operation)
    //   opus_int32 num_encoded_bytes = opus_encode(encoder, encodeBuffer, FRAME_SIZE, opus_output, sizeof(opus_output));
    //   opusDataCharacteristic.notify(opus_output, num_encoded_bytes);
    //   samplesRead += FRAME_SIZE;
    //   digitalWrite(LED_BUILTIN,LOW);
    //   delayMicroseconds(10);
    //   digitalWrite(LED_BUILTIN,HIGH);
    // }

  } else {
    delay(1000); // Reduce CPU usage when not connected
  }
}

// The desktop app sends one complete, checksummed record. A malformed write is
// ignored, leaving the last known-good calibration in effect.
void calibration_write_callback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len != sizeof(CalibrationRecord)) return;
  CalibrationRecord record;
  memcpy(&record, data, sizeof(record));
  if (!saveStoredCalibration(record)) return;
  CalibrationRecord saved = currentCalibrationRecord();
  chr->write((const uint8_t*)&saved, sizeof(saved));
}

void onPDMdata() {
  int bytesAvailable = PDM.available(); //normally this will be FRAME_SIZE*2
  PDM.read(pdmBuffer, bytesAvailable); 
  int samplesReceived = bytesAvailable/2; 
  for(int i=0; i < samplesReceived; i++){
    sampleBuffer[writeIndex] = pdmBuffer[i];
    writeIndex = (writeIndex + 1) % BUFF_SIZE; // Circular buffer
    samplesWritten++;
  }
}
