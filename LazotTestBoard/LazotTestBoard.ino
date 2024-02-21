#include "avr/io.h"
#include "avr/interrupt.h"
#include <PCF8575.h>
#include <SoftwareSerial.h>



#define SENSOR_COUNT (16)

#define TEMPERATURE_SET_POINT_DEFAULT (60)

#define TEMPERATURE_MAX_POINT (500)

// DIP SWITCH
#define DIP_SWITCH_PIN_4 (4)
#define DIP_SWITCH_PIN_3 (5)
#define DIP_SWITCH_PIN_2 (6)
#define DIP_SWITCH_PIN_1 (7)

// PCF8575
#define IO_EXPANDER_I2C_ADDR (0x20)
#define IO_EXPANDER_SCL (A5)
#define IO_EXPANDER_SDA (A4)


// MAX6675 
#define TEMP_SCK_PIN (13)
#define TEMP_SO_PIN (12)

// SERIAL
#define SERIAL_RX_PIN (2)
#define SERIAL_TX_PIN (3)


typedef enum{
  SENSOR_BOARD_NONE=0,
  SENSOR_BOARD_1_TO_16=1,
  SENSOR_BOARD_17_TO_32=2,
  SENSOR_BOARD_33_TO_48=3,
  SENSOR_BOARD_49_TO_64=4,
  SENSOR_BOARD_MAX,
}sensorBoard_t;

typedef enum{
  SENSOR_READ_STATE_NONE=0,
  SENSOR_READ_STATE_START=1,
  SENSOR_READ_STATE_READ=2,
  SENSOR_READ_STATE_WAIT=3,
  SENSOR_READ_STATE_STOP=4,
  SENSOR_READ_STATE_MAX,
}sensorReadState_t;

typedef struct{
  uint8_t id;
  uint8_t csIoNum;
  float tempRead;
  uint16_t tempSet;
  uint16_t halfTime;
  uint16_t fullTime;
  uint16_t endTime;
  uint8_t halTimerFlag;
}sensorIns_t;

typedef struct{
  uint8_t state;
  unsigned long startTime;
  unsigned long deltaTime;
}sensorReadingSm_t;



static uint8_t gBoardNum=SENSOR_BOARD_NONE;
static sensorIns_t gSensorInstances[SENSOR_COUNT]={0};
static sensorReadingSm_t gSensorReadingSm={0};
static uint16_t gProcessTime=0;

static PCF8575 gIoExpander(IO_EXPANDER_I2C_ADDR);
static SoftwareSerial gSerial(SERIAL_RX_PIN,SERIAL_TX_PIN);



static void InitDipSwitchs(void){
  pinMode(DIP_SWITCH_PIN_1,INPUT);
  pinMode(DIP_SWITCH_PIN_2,INPUT);
  pinMode(DIP_SWITCH_PIN_3,INPUT);
  pinMode(DIP_SWITCH_PIN_4,INPUT);
}

static uint8_t SetAllCsState(uint8_t state){
  
  if(!gIoExpander.isConnected()){
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 disconnected");
    return false;
  }
  if(state==HIGH)
    gIoExpander.write16(0xffff);
  else if(state==LOW)
    gIoExpander.write16(0x00);
  return true;
}

static uint8_t SetCsState(uint8_t csIo,uint8_t state){
  
  if(csIo>=SENSOR_COUNT){
    Serial.print(__LINE__);
    Serial.println(" :[ERR] invalid csIo");
    return false;
  }
  if(!gIoExpander.isConnected()){
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 disconnected");
    return false;
  }
  
  gIoExpander.write(csIo,state);
  return true;
}

byte max6675_ReadSpi(void) {
  
  int i;
  byte d = 0;
  
  for (i = 7; i >= 0; i--) {
    digitalWrite(TEMP_SCK_PIN, LOW);
    delayMicroseconds(10);
    if (digitalRead(TEMP_SO_PIN)) {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(TEMP_SCK_PIN, HIGH);
    delayMicroseconds(10);
  }

  return d;
}

static void max6675_Init(void) {
  pinMode(TEMP_SCK_PIN, OUTPUT);
  pinMode(TEMP_SO_PIN, INPUT);
  SetAllCsState(HIGH);
}

float max6675_ReadTemp(uint8_t csIo) {
  uint16_t v;
  
  SetCsState(csIo,LOW);
  delayMicroseconds(10);

  v = max6675_ReadSpi();
  v <<= 8;
  v |= max6675_ReadSpi();

  SetCsState(csIo,HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN;
  }

  v >>= 3;
  return v * 0.25;
}

static uint8_t DetermineBoardNumber(void){
  if(digitalRead(DIP_SWITCH_PIN_1)==LOW){
    return SENSOR_BOARD_1_TO_16;
  }
  else if(digitalRead(DIP_SWITCH_PIN_2)==LOW){
    return SENSOR_BOARD_17_TO_32;
  }
  else if(digitalRead(DIP_SWITCH_PIN_3)==LOW){
    return SENSOR_BOARD_33_TO_48;
  }
  else if(digitalRead(DIP_SWITCH_PIN_4)==LOW){
    return SENSOR_BOARD_49_TO_64;
  }
  else{
    return SENSOR_BOARD_NONE;
  }
}

static void InitializeSensorInstances(void){
  uint8_t i=0,start;
  
  switch(gBoardNum){
    case SENSOR_BOARD_1_TO_16:
    start=1;
    break;
    case SENSOR_BOARD_17_TO_32:
    start=17;
    break;
    case SENSOR_BOARD_33_TO_48:
    start=33;
    break;
    case SENSOR_BOARD_49_TO_64:
    start=49;
    break;
  }

  for(i=0;i<SENSOR_COUNT;i++){
    gSensorInstances[i].id=start;
    gSensorInstances[i].csIoNum=i;
    gSensorInstances[i].tempRead=0.0f;
    gSensorInstances[i].halfTime=0U;
    start++;
  }
}

static void ReadAllSensors(void){
  uint8_t i;
  for(i=0;i<SENSOR_COUNT;i++){   
    gSensorInstances[i].tempRead=max6675_ReadTemp(gSensorInstances[i].csIoNum);
    if(gSensorInstances[i].tempRead>=TEMPERATURE_MAX_POINT)
    gSensorInstances[i].tempRead=NAN;
  }
}

static void PrintAllSensorData(void){
  uint8_t i;
  Serial.println();
  for(i=0;i<SENSOR_COUNT;i++){
    Serial.print("TEMP[ ");
    Serial.print(gSensorInstances[i].id);
    Serial.print("] value : ");
    Serial.println(gSensorInstances[i].tempRead);
  }
}

static void PrintSensorInstances(void){
  uint8_t i;
  Serial.println();
  for(i=0;i<SENSOR_COUNT;i++){
    Serial.print("{ id: ");
    Serial.print(gSensorInstances[i].id);
    Serial.print(" ioNum: ");
    Serial.print(gSensorInstances[i].csIoNum);
    Serial.print(" tempRead: ");
    Serial.print(gSensorInstances[i].tempRead);
    Serial.print(" halfTime: ");
    Serial.print(gSensorInstances[i].halfTime);
    Serial.print(" fullTime: ");
    Serial.print(gSensorInstances[i].fullTime);
    Serial.print(" endTime: ");
    Serial.print(gSensorInstances[i].endTime);
    Serial.println(" }");
  }
}

static void SensorReadingSM(uint8_t state){
  switch(state){
    case SENSOR_READ_STATE_START:{
      Serial.println("Sensor Reading started");
      gSensorReadingSm.state=SENSOR_READ_STATE_READ;
      break;
    }
    case SENSOR_READ_STATE_READ:{
      ReadAllSensors();
      gSensorReadingSm.startTime=millis();
      PrintAllSensorData();
      gSensorReadingSm.state=SENSOR_READ_STATE_WAIT;
      break;
    }
     case SENSOR_READ_STATE_WAIT:{
      gSensorReadingSm.deltaTime=millis()-gSensorReadingSm.startTime;
      if(gSensorReadingSm.deltaTime>= 250) // TODO use macro
      {
        Serial.print("deltaTime: ");
        Serial.println(gSensorReadingSm.deltaTime);
        gSensorReadingSm.state=SENSOR_READ_STATE_READ;
      }
      break;
    }
    case SENSOR_READ_STATE_STOP:{
      // do nothing
      break;
    }
    
    default :{
      Serial.print(__LINE__);
      Serial.println(" :[ERR] invalid state");
      break;
    }
  }
}

static void SetTemperatureSetPointsToDefault(void){
  uint8_t i;
  for(i=0;i<SENSOR_COUNT;i++){
    if(gSensorInstances[i].tempSet<=0){
        gSensorInstances[i].tempSet=TEMPERATURE_SET_POINT_DEFAULT;
    }
  }
}

static void CalculateProcessTime(void){
  float temp=gSensorInstances[SENSOR_COUNT-1].tempRead;
  if(temp!=NAN && (int)temp<TEMPERATURE_MAX_POINT){
    gProcessTime=(435+(325-temp))+(60*((325-temp)/50));// TODO calculate actual process time
    Serial.print("Process Time: ");
    Serial.println(gProcessTime);
  }else{
    gProcessTime=0;
  }
}

void setup() {

  Wire.begin();
  
  Serial.begin(9600);
  Serial.println();

  gSerial.begin(9600);
  gSerial.println("Hello soft serail");

  /*-------------DETECT BOARD NUMBER----------*/
  InitDipSwitchs();
  
  gBoardNum = DetermineBoardNumber();

  while(gBoardNum <= SENSOR_BOARD_NONE 
        || gBoardNum >= SENSOR_BOARD_MAX)
  {
      Serial.println(__LINE__);
      Serial.print(" :[ERR] Invalid Board Number:");
      Serial.println(gBoardNum);
  }
    
  Serial.print("Board number: ");
  Serial.println(gBoardNum);
  /*------------------------------------------*/

  InitializeSensorInstances();

  // TODO read set points from EEPROM

  /*
   * If temperature set point not found in EEPROM
   * set set points to TEMPERATURE_SET_POINT_DEFAULT
   */
  SetTemperatureSetPointsToDefault();
  
  PrintSensorInstances();

  
  while(!gIoExpander.begin()){
   Serial.print(__LINE__);
   Serial.println(" :[ERR] PCF8575 init failed");
  }
  
  max6675_Init();

  // start sensor state machine
  gSensorReadingSm.state=SENSOR_READ_STATE_START;
}

void loop() {
  
  SensorReadingSM(gSensorReadingSm.state);

  // Calculate time value
  CalculateProcessTime();

  // Check for trigger
  uint8_t i;
  for(i=0;i<(SENSOR_COUNT-1);i++){
   if(gSensorInstances[i].tempSet <= 0 )
   {
      Serial.print(__LINE__);
      Serial.println(" :[ERR] invalid set temperature");
      break;
   }else if(gSensorInstances[i].tempRead == NAN){
      Serial.print(__LINE__);
      Serial.println(" :[ERR] invalid temperature");
      break;
   }
   else if((int)gSensorInstances[i].tempRead >= 
            gSensorInstances[i].tempSet)
   {          
    Serial.print("Temperature Set Point Reached :[ ");
    Serial.print(i);
    Serial.println(" ]");
    
    // TODO calculate full time

    // TODO calculate half time

    // TODO calculate end time
    
   }
   
  }
  
  
  
//  startTime=millis();
//  ReadAllSensors();
//  deltaTime=millis()-startTime;
//  Serial.print("Time : ");
//  Serial.print(deltaTime);
//  Serial.println(" ms");
//  PrintAllSensorData();
}
