
//NOTE: Thermopile values are now smoothed   (((Xt-2...)/2 + Xt-1)/2 + X)/2    6/14/17
//ID107 --> D3
/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#include <SPI.h>
#include <BLEPeripheral.h>
#include <BLEUtil.h>
#include <Wire.h>
//#include <ArduinoLowPower.h>
#include <KX022_SPI.h>

#include "BME280I2C.h"

#include "MAX30100.h"

//#include "Adafruit_CCS811.h" // VOC/eCO2 SENSOR
#include <CCS811.h>

//#define ADDR      0x5A
#define ADDR      0x5B
#define WAKE_PIN  4



/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool  debug = true;        //turn serial on/off to get data or turn up sample rate
bool  debug_time = false;    //turn loop component time debug on/off

int   speedHz = 999; //throttled loop speed - native max loop speed is about 35 ms or 28Hz
float speedMs = 1000 / speedHz;  //native max loop speed is 62.5 ms or 16Hz
  
bool HAVE_PULSE_OX_DATA_FLAG = false;

int acc_sample_offset = 0;

/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/
//SCL = 23  SDA = 22    

#define PIN_SERIAL_RX           17
#define PIN_SERIAL_TX           18

#define BATTERY_PIN             1
#define BUTTON_PIN1             4
#define BUTTON_PIN2             7
#define BUZZER_PIN              6  

//Accelerometer Pins
#define CS_PIN                  11

#define KX022_SDI 14
#define KX022_SDO 15
#define KX022_SCL 16
#define KX022_INT 13

//#define PIN_SPI_MISO              (KX022_SDO)
//#define PIN_SPI_MOSI              (KX022_SDI)
//#define PIN_SPI_SCK               (KX022_SCL)

//Thermopile Addresses
#define MLX90615_I2CADDR          0x00
#define MLX90615_I2CADDR1         0x2A

// RAM
#define MLX90615_RAWIR1           0x04
#define MLX90615_RAWIR2           0x05
#define MLX90615_TA               0x26
#define MLX90615_TOBJ1            0x27
#define MLX90615_TOBJ2            0x28
// EEPROM
#define MLX90615_TOMAX            0x20
#define MLX90615_TOMIN            0x21
#define MLX90615_PWMCTRL          0x22
#define MLX90615_TARANGE          0x23
#define MLX90615_EMISS            0x24
#define MLX90615_CONFIG           0x25
#define MLX90615_ADDR             0x0E

//dummy LED pin for BLE
#define LED_PIN                   10


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

  /*
  //LED
  float     blueLED_timer = 0;
  float     greenLED_timer = 0;
  int       LED_counter = 1;
  bool      greenLED_status = false;

  //Heart Rate
  int       heart_rate_counter = 0;
  bool      heart_rate_LED_status = false;
  float     heartSensorValue = 0;
*/
  //OD Detection Condition Defaults
  float tempSetLow = 90;
  float tempSetHigh = 100;
  float tempDeltaSetLow = -4;
  float tempDeltaSetHigh = 4;
  float motionSetLow = 0;
  float motionSetHigh = 0.6;

  
  //Timestamp
  float     clocktime;
  int       secondCounter =     0;
  int       tenSecondCounter =  0;
    
  //Bluetooth
  unsigned long microsPerReading, microsPrevious;
  float     accelScal;
  int       command_value =     0;    //controlls how device and app talk to each other

  //System
  int       varState =          0;    //variable state controlled in app and appended to data stream
  int       buzzerState =       1;    //variable state controlled in app to disable or enable buzzer. Default enabled.
  int       buttonState =       0;    // variable for reading the button status

  //MLX90615 Thermopiles
  float     TObj =              0;
  float     TAmb =              0;
  float     TObjOld =           0;
  float     TObjChange[5] =     {0,0,0,0,0};
  float     TObjTenSec[30] =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float     TObjDeltaLongAv;         //for detection

  
  //KX022 Accelerometer
  // pins used for the connection with the sensor
  // the other you need are controlled by the SPI library):
  //  const int dataReadyPin = 11;

  float     acc[3];
  float     accOld[3];
  float     accMovement;
  float     accMovementChange[5] =  {0,0,0,0,0};
  float     accMovSec[30] =         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float     accMovLongAv;           //for detection
  double    pitch = 0;
  double    roll = 0;
  double    pitchOld = 0;
  double    rollOld = 0;

/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/
//Bluetooth
// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral =   BLEPeripheral();

// create service
//BLEService customService =    BLEService("FFFF");
BLEService customService =      BLEService("a000");

// create command i/o characteristics
BLECharCharacteristic           ReadOnlyArrayGattCharacteristic  = BLECharCharacteristic("a001", BLERead);
BLECharCharacteristic           WriteOnlyArrayGattCharacteristic = BLECharCharacteristic("a002", BLEWrite);

//create streaming data characteristic
BLECharacteristic               DataCharacteristic("a003", BLERead | BLENotify, 20);  //@param data - an Uint8Array.

//create streaming neural network i/o characteristic
BLECharacteristic    ReadSettingsCharacteristic  = BLECharacteristic("a004", BLERead | BLENotify, 20); //@param data - an Uint8Array.
BLECharacteristic    WriteSettingsCharacteristic  = BLECharacteristic("a005", BLEWrite, 20); //@param data - an Uint8Array.



//BME280 barometer and thermometer 
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

//CCS811 carbon dioxide sensor
//Adafruit_CCS811 ccs;

CCS811 ccs;

//KX022 Accelerometer
KX022_SPI kx022(CS_PIN);

//MAX30100 Red and IR Pulse Ox
//MAX30100* pulseOximeter;


/********************************************************************************************************/
/************************ UTILITY FUNCTIONS *************************************************/
/********************************************************************************************************/
float differenceBetweenAngles(float firstAngle, float secondAngle)
  {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }


/********************************************************************************************************/
/************************ MLX90615 THERMOPILE FUNCTIONS *************************************************/
/********************************************************************************************************/

uint16_t read16(uint8_t a) {
  uint8_t _addr = MLX90615_I2CADDR;

 // _addr = MLX90615_I2CADDR;   //thermopile addresses

  
  uint16_t  ret;
  Wire.beginTransmission(_addr);                  // start transmission to device 
  Wire.write(a); delay(1);                        // sends register address to read from
  Wire.endTransmission(false);                    // end transmission
  Wire.requestFrom(_addr, (uint8_t)3); delay(1);  // send data n-bytes read
  ret = Wire.read();// delay(1);                    // receive DATA
  ret |= Wire.read() << 8; delay(1);              // receive DATA
  uint8_t   pec = Wire.read(); delay(1);
  return    ret;
}

float readTemp(uint8_t reg) {
  float temp;
  temp =  read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

double readObjectTempF() {
  return (readTemp(MLX90615_TOBJ1) * 9 / 5) + 32;
}

double readAmbientTempF() {
  return (readTemp(MLX90615_TA) * 9 / 5) + 32;
}

double readObjectTempC() {
  return readTemp(MLX90615_TOBJ1);
}

double readAmbientTempC(int sensorNum) {
  return readTemp(MLX90615_TA);
}


/********************************************************************************************************/
/************************ BLUETOOTH BLE FUNCTIONS *************************************************/
/********************************************************************************************************/
void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler
  command_value = 1;
//  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  }
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  command_value = 0;
//  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Disconnected event, central: "));
    Serial.println(central.address());
  }
}

void blePeripheralServicesDiscoveredHandler(BLECentral& central) {
  // central  services discovered event handler
  if(debug){
    Serial.print(F(" services discovered event, central: "));
    Serial.println(central.address());
  }
/*
  if (ReadOnlyArrayGattCharacteristic.canRead()) {
    Serial.println(F("ReadOnlyArrayGattCharacteristic"));
    ReadOnlyArrayGattCharacteristic.read();
  }

  if (WriteOnlyArrayGattCharacteristic.canWrite()) {
    Serial.println(F("WriteOnlyArrayGattCharacteristic"));

   // unsigned long writeValue = 42;
    static uint8_t writeValue[10] = {0};
  //  writeValue[0] = 5;

    WriteOnlyArrayGattCharacteristic.write((const unsigned char*)&writeValue, sizeof(writeValue));
  } */
}

void bleCharacteristicValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  
    if(debug){ Serial.print(F(" Begin bleCharacteristicValueUpdatedHandle: ")); }
    
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
 // char char_buf[2]={0,0};
  //int command_value;
  
  String bleRawVal = "";
  for (byte i = 0; i < the_length; i++){ 
    bleRawVal += String(the_buffer[i], HEX); 
  }

  char *char_buf = const_cast<char*>(bleRawVal.c_str());
  
  command_value = (int)strtol(char_buf, NULL, 16);

//  bleRawVal.toCharArray(temp_char_buffer, the_length);
 // sscanf(temp_char_buffer, "%x", &command_value);

//  if(debug) Serial.print("Raw command: "); Serial.println( the_buffer );
//  if(debug) 
  if(debug){ Serial.print("APP COMMAND: "); Serial.println( command_value ); }



  BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength());
  delay(1);
}

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  if(debug) Serial.print(F("Characteristic event, written: "));

  if (ReadOnlyArrayGattCharacteristic.value()) {
    if(debug) Serial.println(F("LED on"));

  } else {
    if(debug) Serial.println(F("LED off"));
  }
}

void bleSettingsUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
  
  String bleRawVal = "";
    for (byte i = 0; i < the_length; i++){ 
    String bleRawVal_temp = String(the_buffer[i], HEX); 
    if(bleRawVal_temp.length() == 1)  bleRawVal += String("0");
    bleRawVal += bleRawVal_temp;
  }
  
 if(debug){ Serial.print("BLERAWVAL: "); Serial.println(bleRawVal); }

  char settings_index_buffer1[3];
  char settings_index_buffer2[3];
  char settings_index_buffer3[3];
  char settings_index_buffer4[3];
  char settings_index_buffer5[3];
  char settings_index_buffer6[3];
  char settings_index_buffer7[3];
  char settings_index_buffer8[3];
  char settings_index_buffer9[3];
  char settings_index_buffer10[3];
  char settings_index_buffer11[3];
  char settings_index_buffer12[3];
  char settings_index_buffer13[3];
  char settings_index_buffer14[3];

  bleRawVal.substring(0,2).toCharArray(settings_index_buffer1, 3);
  bleRawVal.substring(2,4).toCharArray(settings_index_buffer2, 3);
  bleRawVal.substring(4,6).toCharArray(settings_index_buffer3, 3);
  bleRawVal.substring(6,8).toCharArray(settings_index_buffer4, 3);
  bleRawVal.substring(8,10).toCharArray(settings_index_buffer5, 3);
  bleRawVal.substring(10,12).toCharArray(settings_index_buffer6, 3);
  bleRawVal.substring(12,14).toCharArray(settings_index_buffer7, 3);
  bleRawVal.substring(14,16).toCharArray(settings_index_buffer8, 3);
  bleRawVal.substring(16,18).toCharArray(settings_index_buffer9, 3);
  bleRawVal.substring(18,20).toCharArray(settings_index_buffer10, 3);
  bleRawVal.substring(20,22).toCharArray(settings_index_buffer11, 3);
  bleRawVal.substring(22,24).toCharArray(settings_index_buffer12, 3);
  bleRawVal.substring(24,26).toCharArray(settings_index_buffer13, 3);
  bleRawVal.substring(26,28).toCharArray(settings_index_buffer14, 3);

  int settings_num1 = (int)strtol(settings_index_buffer1, NULL, 16);
  int settings_num2 = (int)strtol(settings_index_buffer2, NULL, 16);
  float settings_num3 = (float)strtol(settings_index_buffer3, NULL, 16);
  float settings_num4 = (float)strtol(settings_index_buffer4, NULL, 16);
  float settings_num5 = (float)strtol(settings_index_buffer5, NULL, 16);
  float settings_num6 = (float)strtol(settings_index_buffer6, NULL, 16);
  float settings_num7 = (float)strtol(settings_index_buffer7, NULL, 16);
  float settings_num8 = (float)strtol(settings_index_buffer8, NULL, 16);
  float settings_num9 = (float)strtol(settings_index_buffer9, NULL, 16);
  float settings_num10 = (float)strtol(settings_index_buffer10, NULL, 16);
  float settings_num11 = (float)strtol(settings_index_buffer11, NULL, 16);
  float settings_num12 = (float)strtol(settings_index_buffer12, NULL, 16);
  float settings_num13 = (float)strtol(settings_index_buffer13, NULL, 16);
  float settings_num14 = (float)strtol(settings_index_buffer14, NULL, 16);

  float tempChangeRangeMin = 5;

  tempSetLow = (settings_num1*100 + settings_num2)/2 + 60;
  tempSetHigh = (settings_num3*100 + settings_num4)/2 + 60;
  tempDeltaSetLow = (settings_num5*100 + settings_num6)/15 - tempChangeRangeMin;
  tempDeltaSetHigh = (settings_num7*100 + settings_num8)/15 - tempChangeRangeMin;
  motionSetLow = (settings_num9*100 + settings_num10)/15;
  motionSetHigh = (settings_num11*100 + settings_num12 )/15;

  if(debug){ Serial.print("settings_num13: "); Serial.println( settings_num13 ); }
  if(debug){ Serial.print("settings_num14: "); Serial.println( settings_num14 ); }

  buzzerState = settings_num14;

/*   if(debug){
  Serial.print("BASE10: "); Serial.print( (settings_num1*100 + settings_num2)/2 + 60 ); 
  Serial.print(" , "); Serial.print( (settings_num3*100 + settings_num4)/2 + 60 );
  Serial.print(" , "); Serial.print( (settings_num5*100 + settings_num6)/15 - tempChangeRangeMin);
  Serial.print(" , "); Serial.print( (settings_num7*100 + settings_num8)/15 - tempChangeRangeMin);
  Serial.print(" , "); Serial.print( (settings_num9*100 + settings_num10)/15 );
  Serial.print(" , "); Serial.println(  (settings_num11*100 + settings_num12 )/15 );
   }  */
  
}

/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{
    Serial.begin(115200); 
    if(debug) Serial.print("STARTING\t");
    delay(50);

    // start the I2C library:
    Wire.begin();
    delay(50);

    //Configure button pins
    pinMode(BUTTON_PIN1, INPUT); 
    pinMode(BUTTON_PIN2, INPUT);

    //Configure buzzer pin
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, 0); 

  /************ INIT BME280 BAROMETER THERMOMETER ********************/
    if(debug) Serial.println(F("BMP280 INIT"));
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }

  /************ INIT CCS811 CARBON DIOXIDE SENSOR ********************/
    Serial.println("CCS811 test");

  if(!ccs.begin(uint8_t(ADDR), uint8_t(WAKE_PIN)))
    Serial.println("Initialization failed.");
  /*
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
   // while(1);
  }
  */

  //calibrate temperature sensor
//  while(!ccs.available());
//  float temp = ccs.calculateTemperature();
//  ccs.setTempOffset(temp - 25.0);
  
  /************ INIT KX022 ACCELEROMETER *****************************/
    if(debug) Serial.print("KX022 INIT RESPONSE WAS ");
    Serial.println(kx022.init());
    delay(1000);

  /************ INIT MAX30100 PULSE OX *******************************/
 //   if(debug) Serial.print("MAX30100 INIT");
//    pulseOximeter = new MAX30100();
//    delay(1000);

  /************** INIT BLUETOOTH BLE instantiate BLE peripheral *********/
    // set LED pin to output mode
   // set advertised local name and service UUID
    blePeripheral.setLocalName("ODAlert");
    blePeripheral.setDeviceName("ODAlert");
    blePeripheral.setAdvertisedServiceUuid(customService.uuid());
    blePeripheral.setAppearance(0xFFFF);
  
    // add attributes (services, characteristics, descriptors) to peripheral
    blePeripheral.addAttribute(customService);
    
    blePeripheral.addAttribute(ReadOnlyArrayGattCharacteristic);
    blePeripheral.addAttribute(WriteOnlyArrayGattCharacteristic);
    
    blePeripheral.addAttribute(DataCharacteristic); //streaming data for app graph

    blePeripheral.addAttribute(ReadSettingsCharacteristic); //OD Detection settings etc.
    blePeripheral.addAttribute(WriteSettingsCharacteristic); //OD Detection settings etc.

    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  //  blePeripheral.setEventHandler(BLEWritten, blePeripheralServicesDiscoveredHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    ReadSettingsCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleSettingsUpdatedHandle);
    WriteSettingsCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleSettingsUpdatedHandle);
    
    // assign initial values
    char readValue[10] = {0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[10] = {0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);

    char readSettingsValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ReadSettingsCharacteristic.setValue(readSettingsValue);
    char writeSettingsValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    WriteSettingsCharacteristic.setValue(writeSettingsValue);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
  
    // begin initialization
    blePeripheral.begin();
  
    if(debug) Serial.println("BLE MOBILE APP PERIPHERAL");

  delay(500);  
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     

  /********************************************************************************** 
  ******************** LOOP SPEED CONTROL *******************************************
  ***********************************************************************************/
 if(clocktime + speedMs < millis()){
    /***************** Timestamp ****************************************************/
    clocktime = millis();
    if(debug){ Serial.println(" "); Serial.print("TIME: "); Serial.print( clocktime/1000 ); Serial.println(" s"); }
    if(debug_time){ Serial.print("Time after init speed limit check: "); Serial.println(millis() - clocktime); }


  /******************* BMP280 BAROMETER THERMOMETER *********************************/
   printBME280Data(&Serial);
   delay(100);

  /******************* CCS811 VOC/eCO2 SENSOR ***************************************/
 //  sampleCarbonDioxide();
 //  delay(100);
   //sensor.compensate(22.56, 30.73);  // replace with t and rh values from sensor
  ccs.getData();
  Serial.print("CO2 concentration : "); Serial.print(ccs.readCO2()); Serial.println(" ppm");
  Serial.print("TVOC concentration : "); Serial.print(ccs.readTVOC()); Serial.println(" ppb");
  Serial.println();
  delay(1000);
    

  /******************* MAX30100 PULSE OXIMETRY **************************************/
  //  samplePulse();

  /******************* Bluetooth Connection Poll ************************************/
    blePeripheral.poll();   if(debug_time){ Serial.print("Time after BLE poll: "); Serial.println( (millis() - clocktime))/1000; }

  /** BLOCKING SPEED CONTROL LOOP ***/
if(acc_sample_offset <= 0){
    
  /******************* Read KX022 Accelerometer *************************************/
    sampleAngularPosition();

  /******************* Read MLX90615 Thermopile Sensor ******************************/
    sampleThermopiles();
    
  /******************* Print Data to Console for Debugging **************************/    
//    printSensorData();

   /** BLOCKING SPEED CONTROL LOOP ***/
acc_sample_offset = 1;
} else { acc_sample_offset--; }

  /*********** Data Processing and Delta *******************/
//deltaProcessing();

  /*********** Bluetooth Data Transmittion *******************/
    transmitSensorData();

    if(debug_time){ Serial.print("TIME LOOP: "); Serial.print(millis() - clocktime); Serial.print("\tSeconds: "); Serial.print(secondCounter); }
        
  } //end loop speed
} //end infinate loop



/********************************************************************************************************/
/************************ FUNCTIONS *********************************************************************/
/********************************************************************************************************/

/*********************************************************************
*************** READ MAX30100 PULSE OXIMETRY *************************
*********************************************************************/
/*
void samplePulse(){
    //You have to call update with frequency at least 37Hz. But the closer you call it to 100Hz the better, the filter will work.
    pulseoxymeter_t result = pulseOximeter->update();
    
  
    if( result.pulseDetected == true )
    {
    HAVE_PULSE_OX_DATA_FLAG = true;  
    
      Serial.println("BEAT");
      
      Serial.print( "BPM: " );
      Serial.print( result.heartBPM );
      Serial.print( " | " );
    
      Serial.print( "SaO2: " );
      Serial.print( result.SaO2 );
      Serial.println( "%" );
  
      Serial.print("{P2|BPM|255,40,0|");
      Serial.print(result.heartBPM);
      Serial.print("|SaO2|0,0,255|");
      Serial.print(result.SaO2);
      Serial.println("}");
  
      
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      Serial.println("<<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>     <<^^>>");
      
    } else { HAVE_PULSE_OX_DATA_FLAG = false; }
    
  if(debug_time){ Serial.print("Time after pulse ox update: "); Serial.println( (millis() - clocktime)/1000); }
}
*/


/*********************************************************************
*************** READ KX022 ACCELEROMETER *****************************
*********************************************************************/
void sampleAngularPosition(){
    //KX022 ACCELEROMETER I2C
    acc[0] = (float)(10 * kx022.getAccel(0) );
    acc[1] = (float)(10 * kx022.getAccel(1) );
    acc[2] = (float)(10 * kx022.getAccel(2) );
    float eulerX, eulerY, eulerZ;
    
    eulerX = (float)kx022.getAccel(0); eulerY = (float)kx022.getAccel(1); eulerZ = (float)kx022.getAccel(2); 
    pitch = (180/3.141592) * ( atan2( eulerX, sqrt( eulerY * eulerY + eulerZ * eulerZ)) );
    roll = (180/3.141592) * ( atan2(-eulerY, -eulerZ) );

    //adjust for device upward = 180 to prevent crossover 
    pitch = pitch + 180;

    //adjust for gap
    if(pitch < 30){ 
      pitch = pitch + 330;
    } else {
      pitch = pitch - 30; 
    }

    roll = roll + 180;

    //adjust for gap
    if(roll < 60){ 
      pitch = pitch + 300;
    } else {
      pitch = pitch - 60; 
    }

    acc_sample_offset = 10; //sample offset, sample every X loops
    
    /************** Update Historical Data and Change *****************/
    accMovement = abs(pitchOld - pitch) + abs(rollOld - roll);
    pitchOld = pitch;
    rollOld = roll;
    if(debug_time){ Serial.print("Time after KX022 Angular Position: "); Serial.println( (millis() - clocktime))/1000; }
}

/*********************************************************************
*************** READ MLX90615 THERMOPILES ****************************
*********************************************************************/
void sampleThermopiles(){
    TAmb = readAmbientTempF(); 
    TObj = readObjectTempF();
    if(debug_time){ Serial.print("Time after thermo read: "); Serial.println(millis() - clocktime); }
}


/*********************************************************************
*************** TRANSMIT SENSOR DATA OVER BLUETOOTH ****************** 
*********************************************************************/
void transmitSensorData(){
 //  if(HAVE_PULSE_OX_DATA_FLAG){

          BLECentral central = blePeripheral.central();
          
          if(central){ // if a central is connected to peripheral

              /*
               * reduce temperture readings to a range between 0 and 31, then multiply to use up the 256 max decimal value of an 8 bit integer
               * Object temperature floor: 70F
               * Object temperature ceiling: 101F
               */
              float TObj_compressed = TObj;
              if(TObj_compressed < 70){   TObj_compressed = 70;  }
              if(TObj_compressed > 111){  TObj_compressed = 110;  }
              TObj_compressed = (TObj_compressed - 70)*6;
              
              
              float TAmb_compressed;
              TAmb_compressed = TAmb;
              if(TAmb < 70){ TAmb_compressed = 70;  }
              if(TAmb > 111){ TAmb_compressed = 110;  }
              TAmb_compressed = (TAmb_compressed - 70)*6;



              const unsigned char imuCharArray[20] = {
                  (uint8_t)TObj_compressed,              //1
                  (uint8_t)TAmb_compressed,              //empty
                  (uint8_t)(TObjDeltaLongAv * 10),
                  (uint8_t)0,
                  (uint8_t)( (acc[0] + 1) * 100),        //5
                  (uint8_t)( (acc[1] + 1) * 100),
                  (uint8_t)( (acc[2] + 1) * 100),
                  (uint8_t)( pitch/1.4 ),
                  (uint8_t)( roll/1.4 ),
                  (uint8_t)( accMovLongAv * 100),        //10
                  (uint8_t)0,
                  (uint8_t)0, //(red / 2),
                  (uint8_t)0, //(_IR1 / 2),                  
                  (uint8_t)0, //(_IR2 / 2),   //14
                  (uint8_t)0, 
                  (uint8_t)0,               
                  (uint8_t)0,
                  (uint8_t)0,  
                  (uint8_t)0,
                  (uint8_t)0    //20
              };
            //  imuChar.setValue(imuCharArray, 12); //notify central with new data 
              //send data over bluetooth
              DataCharacteristic.setValue(imuCharArray,20);
              //time to send
              delay(1);
          }  
 //    }

     if(debug_time){ Serial.print("Time after bluetooth send: "); Serial.println(millis() - clocktime); }
}

/*********************************************************************
*************** PRINT SENSOR DATA TO CONSOLE *************************
*********************************************************************/
void printSensorData(){
    Serial.print("ACC X Y Z: "); Serial.print( acc[0] ); Serial.print("F\t  "); 
    Serial.print( acc[1] ); Serial.print("F\t  "); 
    Serial.print( acc[2] ); Serial.println("F"); 

    Serial.print("OT: "); Serial.print( TObj );
    Serial.print("    AT: "); Serial.println( TAmb );
    
    Serial.print("Movement av/10s: "); Serial.println(accMovLongAv);

    Serial.print("motionSetLow: "); Serial.print( motionSetLow ); Serial.println("cm/s"); 
    Serial.print("motionSetHigh: "); Serial.print( motionSetHigh ); Serial.println("cm/s"); 
    Serial.print("buzzerState: "); Serial.println(buzzerState);

        //Debug var state
/*    if(debug){
      Serial.print("**COMMAND: ");
      Serial.println(command_value);
    } */
}

/*********************************************************************
**************** TRACK DATA CHANGE OVER TIME *************************
*********************************************************************/
void deltaProcessing(){
     //RUN ONCE A SECOND
     if( (clocktime/1000) > secondCounter ){
          secondCounter++;

          //update array of time series movement sensor values
          for(int m=29; m>0;m--){ accMovSec[m] = accMovSec[m-1]; }
          accMovSec[0] = accMovement;

          //long period average value for OD detection
          accMovLongAv = 0;
          for(int h=0;h<20;h++){ 
              accMovLongAv = accMovLongAv + accMovSec[h]; 
          }
          if(secondCounter < 20){accMovLongAv = accMovLongAv / secondCounter; }
          else{accMovLongAv = accMovLongAv / 20; }  //OK, so 20 works better than 10

     }
if(debug_time){ Serial.print("Time after data Delta tracking: "); Serial.println(millis() - clocktime); }    

}

/*********************************************************************
**************** BUTTON MGMT *****************************************
*********************************************************************/
void buttonMGMT(){
    

}



/*********************************************************************
*************** ETC **************************************************
*********************************************************************/

void __delay(uint32_t timeout)
{
  uint32_t start;
  start = millis();

 do
 {
   __WFE();
 }
   while ((millis() - start) >= timeout);
}


int hex_to_int(char c){
  int first;
  int second;
  int value;
  
  if (c >= 97) {
    c -= 32;
  }
  first = c / 16 - 3;
  second = c % 16;
  value = first * 10 + second;
  if (value > 9) {
    value--;
  }
  return value;
}

int hex_to_ascii(char c, char d){
  int high = hex_to_int(c) * 16;
  int low = hex_to_int(d);
  return high+low;
}


//////////////////////////////////////////////////////////////////
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");

   delay(1);
}

/*********************************************************************
*************** READ CCS811 VOC/eCO2 SENSOR **************************
*********************************************************************/
/*
void sampleCarbonDioxide(){
  if(ccs.available()){
    float temp = ccs.calculateTemperature();
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.print(ccs.getTVOC());
      Serial.print("ppb   Temp:");
      Serial.println(temp);
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }
}
*/
