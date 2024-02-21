#include "avr/io.h"
#include "avr/interrupt.h"

#include "PCF8575.h"

#define SENSOR_COUNT (16)

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

typedef enum
{
  SENSOR_BOARD_NONE = 0,
  SENSOR_BOARD_1_TO_16 = 1,
  SENSOR_BOARD_17_TO_32 = 2,
  SENSOR_BOARD_33_TO_48 = 3,
  SENSOR_BOARD_49_TO_64 = 4,
  SENSOR_BOARD_MAX,
} sensorBoard_t;

typedef enum
{
  SENSOR_READ_STATE_NONE = 0,
  SENSOR_READ_STATE_START = 1,
  SENSOR_READ_STATE_READ = 2,
  SENSOR_READ_STATE_WAIT = 3,
  SENSOR_READ_STATE_STOP = 4,
  SENSOR_READ_STATE_MAX,
} sensorReadState_t;

typedef struct
{
  uint8_t id;
  uint8_t csIoNum;
  float temperature;
  int timeValue;
} sensorIns_t;

typedef struct
{
  uint8_t state;
  unsigned long startTime;
  unsigned long deltaTime;
} sensorReadingSm_t;

static uint8_t gBoardNum = SENSOR_BOARD_NONE;
static sensorIns_t gSensorInstances[SENSOR_COUNT] = {0};
static PCF8575 gIoExpander(IO_EXPANDER_I2C_ADDR);
static sensorReadingSm_t gSensorReadingSm = {0};

static void InitDipSwitchs(void)
{
  pinMode(DIP_SWITCH_PIN_1, INPUT);
  pinMode(DIP_SWITCH_PIN_2, INPUT);
  pinMode(DIP_SWITCH_PIN_3, INPUT);
  pinMode(DIP_SWITCH_PIN_4, INPUT);
}

static uint8_t SetAllCsState(uint8_t state)
{

  if (!gIoExpander.isConnected())
  {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 disconnected");
    return false;
  }
  if (state == HIGH)
    gIoExpander.write16(0xffff);
  else if (state == LOW)
    gIoExpander.write16(0x00);
  return true;
}

static uint8_t SetCsState(uint8_t csIo, uint8_t state)
{

  if (csIo >= SENSOR_COUNT)
  {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] invalid csIo");
    return false;
  }
  if (!gIoExpander.isConnected())
  {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 disconnected");
    return false;
  }

  gIoExpander.write(csIo, state);
  return true;
}

byte max6675_ReadSpi(void)
{

  int i;
  byte d = 0;

  for (i = 7; i >= 0; i--)
  {
    digitalWrite(TEMP_SCK_PIN, LOW);
    delayMicroseconds(10);
    if (digitalRead(TEMP_SO_PIN))
    {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(TEMP_SCK_PIN, HIGH);
    delayMicroseconds(10);
  }

  return d;
}

static void max6675_Init(void)
{
  pinMode(TEMP_SCK_PIN, OUTPUT);
  pinMode(TEMP_SO_PIN, INPUT);
  SetAllCsState(HIGH);
}

float max6675_ReadTemp(uint8_t csIo)
{
  uint16_t v;

  SetCsState(csIo, LOW);
  delayMicroseconds(10);

  v = max6675_ReadSpi();
  v <<= 8;
  v |= max6675_ReadSpi();

  SetCsState(csIo, HIGH);

  if (v & 0x4)
  {
    // uh oh, no thermocouple attached!
    return NAN;
  }

  v >>= 3;
  return v * 0.25;
}

static uint8_t DetermineBoardNumber(void)
{
  if (digitalRead(DIP_SWITCH_PIN_1) == LOW)
  {
    return SENSOR_BOARD_1_TO_16;
  }
  else if (digitalRead(DIP_SWITCH_PIN_2) == LOW)
  {
    return SENSOR_BOARD_17_TO_32;
  }
  else if (digitalRead(DIP_SWITCH_PIN_3) == LOW)
  {
    return SENSOR_BOARD_33_TO_48;
  }
  else if (digitalRead(DIP_SWITCH_PIN_4) == LOW)
  {
    return SENSOR_BOARD_49_TO_64;
  }
  else
  {
    return SENSOR_BOARD_NONE;
  }
}

static void InitializeSensorInstances(void)
{
  uint8_t i = 0, start;

  switch (gBoardNum)
  {
  case SENSOR_BOARD_1_TO_16:
    start = 1;
    break;
  case SENSOR_BOARD_17_TO_32:
    start = 17;
    break;
  case SENSOR_BOARD_33_TO_48:
    start = 33;
    break;
  case SENSOR_BOARD_49_TO_64:
    start = 49;
    break;
  }

  for (i = 0; i < SENSOR_COUNT; i++)
  {
    gSensorInstances[i].id = start;
    gSensorInstances[i].csIoNum = i;
    gSensorInstances[i].temperature = 0.0f;
    gSensorInstances[i].timeValue = 0U;
    start++;
  }
}

static void ReadAllSensors(void)
{
  uint8_t i;
  for (i = 0; i < SENSOR_COUNT; i++)
  {
    gSensorInstances[i].temperature = max6675_ReadTemp(gSensorInstances[i].csIoNum);
  }
}

static void PrintAllSensorData(void)
{
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print("TEMP[ ");
    Serial.print(i);
    Serial.print("] value : ");
    Serial.println(gSensorInstances[i].temperature);
  }
}

static void PrintSensorInstances(void)
{
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print("{ id: ");
    Serial.print(gSensorInstances[i].id);
    Serial.print(" ioNum: ");
    Serial.print(gSensorInstances[i].csIoNum);
    Serial.print(" temperature: ");
    Serial.print(gSensorInstances[i].temperature);
    Serial.print(" timeValue: ");
    Serial.print(gSensorInstances[i].timeValue);
    Serial.println(" }");
  }
}

static void SensorReadingSM(uint8_t state)
{
  switch (state)
  {
  case SENSOR_READ_STATE_START:
  {
    gSensorReadingSm.state = SENSOR_READ_STATE_READ;
    break;
  }
  case SENSOR_READ_STATE_READ:
  {
    ReadAllSensors();
    gSensorReadingSm.startTime = millis();
    gSensorReadingSm.state = SENSOR_READ_STATE_WAIT;
    break;
  }
  case SENSOR_READ_STATE_WAIT:
  {
    gSensorReadingSm.deltaTime = millis() - gSensorReadingSm.startTime;
    if (gSensorReadingSm.deltaTime >= 250) // TODO use macro
    {
      Serial.print("deltaTime: ");
      Serial.println(gSensorReadingSm.deltaTime);
      gSensorReadingSm.state = SENSOR_READ_STATE_READ;
    }
    break;
  }
  case SENSOR_READ_STATE_STOP:
  {
    // do nothing
    break;
  }

  default:
  {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] invalid state");
    break;
  }
  }
}

void setup()
{

  Wire.begin();

  Serial.begin(9600);
  Serial.println();

  /*-------------DETECT BOARD NUMBER----------*/
  InitDipSwitchs();

  gBoardNum = DetermineBoardNumber();

  while (gBoardNum <= SENSOR_BOARD_NONE || gBoardNum >= SENSOR_BOARD_MAX)
  {
    Serial.println(__LINE__);
    Serial.print(" :[ERR] Invalid Board Number:");
    Serial.println(gBoardNum);
  }

  Serial.print("Board number: ");
  Serial.println(gBoardNum);
  /*------------------------------------------*/

  InitializeSensorInstances();

  PrintSensorInstances();

  while (!gIoExpander.begin())
  {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 init failed");
  }

  max6675_Init();

  // start sensor state machine
  gSensorReadingSm.state = SENSOR_READ_STATE_START;
}

void loop()
{

  SensorReadingSM(gSensorReadingSm.state);

  //  startTime=millis();
  //  ReadAllSensors();
  //  deltaTime=millis()-startTime;
  //  Serial.print("Time : ");
  //  Serial.print(deltaTime);
  //  Serial.println(" ms");
  //  PrintAllSensorData();
}
