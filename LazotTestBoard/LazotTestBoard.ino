#include "avr/io.h"
#include "avr/interrupt.h"

#include "PCF8575.h"
#include "max6675.h"


// PCF8575
#define IO_EXPANDER_I2C_ADDR (0x20)
#define IO_EXPANDER_SCL (A3)
#define IO_EXPANDER_SDA (A4)


// SPI GPIOS
#define TEMP_SCK_PIN (13)
#define TEMP_SO_PIN (12)
#define TEMP_CS_PIN (10)



typedef struct{
  uint16_t readSensor:1;
}flags_t;

//static flags_t gFlags=0;

static PCF8575 gIoExpander(IO_EXPANDER_I2C_ADDR);
static int8_t gSensorToRead=-1;

static uint8_t SetCsState(uint8_t state){
  if(gSensorToRead>15&&gSensorToRead<0){
    Serial.println("invalid sensor selection");
  }
  gIoExpander.write(gSensorToRead,state);
}



/*-------------MAX6675_FUNCTIONS-------------*/

byte ReadSPI(void) {
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


void InitTemperature(void) {
  
  pinMode(TEMP_SCK_PIN, OUTPUT);
  pinMode(TEMP_SO_PIN, INPUT);

//  digitalWrite(TEMP_CS_PIN, HIGH);
  SetCsState(HIGH);
}


float ReadTemperatureInDegC(void) {
  uint16_t v;

  SetCsState(LOW);
  delayMicroseconds(10);

  v = ReadSPI();
  v <<= 8;
  v |= ReadSPI();

  SetCsState(HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN;
  }

  v >>= 3;
  return v * 0.25;
}

float ReadTemperatureInDegF(void) {
  return ReadTemperatureInDegC() * 9.0 / 5.0 + 32; 
 }


typedef enum{
  SENSOR_READING_STATE_IDLE=0,
  SENSOR_READING_STATE_READ=1,
  SENSOR_READING_STATE_WAIT=2,
  SENSOR_READING_STATE_MAX,
}sensorReadingState_t;




#define DEFAULT_SENSOR_CONFIG() 

typedef struct{
  uint8_t id;
  float temperature;
  int timeValue;
}sensorIns_t;

static sensorIns_t gSensorInstances[16]={0};

/*-------------------------------------------*/

static void InitializeSensors(void){
  uint8_t i;
}




static void TurnOnAllGpios(){
  if(gIoExpander.isConnected()){
//    gIoExpander.write16(15);
    gIoExpander.write(15,HIGH);
  }else{
    Serial.println("PCF8575 disconnected");
  }
}

static void TurnOffAllGpios(){
  if(gIoExpander.isConnected()){
//    gIoExpander.write16(15);
    gIoExpander.write(15,LOW);
  }else{
    Serial.println("PCF8575 disconnected");
  }
}


static void InitTimer(void){
  
}

static void StartTimer(void){
  
}

static void StopTimer(void){
  
}



static void SensorReadingSM(uint8_t state){
  switch(state){
    case SENSOR_READING_STATE_READ:{
      Serial.println("SENSOR_READING_STATE_READ");
      break;
    }
    case SENSOR_READING_STATE_WAIT:{
      Serial.println("SENSOR_READING_STATE_WAIT");
      break;
    }
    default :{
      Serial.println("idle state");
      break;
    }
  }
}

typedef enum{
  SENSOR_BOARD_NONE=0,
  SENSOR_BOARD_1_TO_16=1,
  SENSOR_BOARD_17_TO_32=2,
  SENSOR_BOARD_33_TO_48=3,
  SENSOR_BOARD_49_TO_64=4,
  SENSOR_BOARD_MAX,
}sensorBoard_t;


#define DIP_SWITCH_PIN_4 (4)
#define DIP_SWITCH_PIN_3 (5)
#define DIP_SWITCH_PIN_2 (6)
#define DIP_SWITCH_PIN_1 (7)

static uint8_t gBoardNum=SENSOR_BOARD_NONE;

static void InitDipSwitchs(void){
  pinMode(DIP_SWITCH_PIN_1,INPUT);
  pinMode(DIP_SWITCH_PIN_2,INPUT);
  pinMode(DIP_SWITCH_PIN_3,INPUT);
  pinMode(DIP_SWITCH_PIN_4,INPUT);
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

void setup() {
  Serial.begin(9600);

  InitDipSwitchs();
  gBoardNum= DetermineBoardNumber();

  while(gBoardNum <= SENSOR_BOARD_NONE 
  || gBoardNum >= SENSOR_BOARD_MAX){
      Serial.print("[ERROR] Invalid Board Number: ");
      Serial.println(gBoardNum);
  }
  
  Serial.print("Board number: ");
  Serial.println(gBoardNum);
  
  
//
//  Wire.begin();
//  
//  if(!gIoExpander.begin()){
//    Serial.println("PCF8575 init failed");
//  }
//  Serial.println("PCF8575 initialized");
//  
//  InitTemperature();


}



void loop() {

//  SensorReadingSM(gSensorReadingState);
  
  // put your main code here, to run repeatedly:
  // float temp;

  // uint8_t i;
  // for(i=0;i<16;i++){
  //   gSensorToRead=i;
  //   temp=ReadTemperatureInDegC();
  //   Serial.print("Reading sensor[ ");
  //   Serial.print(i);
  //   Serial.print("] value : ");
  //   Serial.print(temp);
  //   Serial.println(" degC");
  //   delay(500);
  // }
  //  Serial.println("\r\n");
  // gSensorToRead=0;
}
