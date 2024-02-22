#include "avr/io.h"
#include "avr/interrupt.h"
#include <PCF8575.h>
#include <SoftwareSerial.h>

#define SENSOR_COUNT (16)

#define TEMPERATURE_SET_POINT_DEFAULT (33)

#define TEMPERATURE_MAX_POINT (500)

#define ADDITIONAL_TIME (120) // 120 sec or 2 minutes

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



// BUZZER
#define BUZZER_PIN (8)

#define BUZZER_RUN_TIME (10)

// SERIAL
#define SERIAL_RX_PIN (2)
#define SERIAL_TX_PIN (3)
#define SERIAL_BAUD_RATE (9600)
#define SERIAL_BUFF_SIZE (256+1)


typedef enum {
  SENSOR_BOARD_NONE = 0,
  SENSOR_BOARD_1_TO_16 = 1,
  SENSOR_BOARD_17_TO_32 = 2,
  SENSOR_BOARD_33_TO_48 = 3,
  SENSOR_BOARD_49_TO_64 = 4,
  SENSOR_BOARD_MAX,
} sensorBoard_t;

typedef enum {
  SENSOR_READ_STATE_NONE = 0,
  SENSOR_READ_STATE_START = 1,
  SENSOR_READ_STATE_READ = 2,
  SENSOR_READ_STATE_WAIT = 3,
  SENSOR_READ_STATE_STOP = 4,
  SENSOR_READ_STATE_MAX,
} sensorReadState_t;

typedef enum {
  SENSOR_TIMER_STATE_NONE = 0,
  SENSOR_TIMER_STATE_START = 1,
  SENSOR_TIMER_STATE_WAIT = 2,
  SENSOR_TIMER_STATE_HALF_TIME = 3,
  SENSOR_TIMER_STATE_FULL_TIME = 4,
  SENSOR_TIMER_STATE_END_TIME = 5,
  SENSOR_TIMER_STATE_STOP = 6,
  SENSOR_TIMER_STATE_MAX,
} sensorTimerState_t;

typedef enum {
  BUZZER_STATE_NONE = 0,
  BUZZER_STATE_START = 1,
  BUZZER_STATE_WAIT = 2,
  BUZZER_STATE_STOP = 3,
  BUZZER_STATE_MAX,
} buzzerState_t;

typedef struct {
  uint8_t id;
  uint8_t csIoNum;
  float tempRead;
  uint16_t tempSet;
  unsigned long startTimeMs;
  unsigned long runTimeMs;
  uint8_t timerState;
  struct {
    uint8_t halfTime: 1;
    uint8_t fullTime: 1;
    uint8_t endTime: 1;
  } flags;
} sensorIns_t;

typedef struct {
  uint8_t state;
  unsigned long startMs;
  unsigned long deltaMs;
} sensorReadingSm_t;

typedef struct {
  uint8_t state;
  unsigned long startMs;
  unsigned long deltaMs;
} buzzerSm_t;

typedef struct {
  char dataBuff[SERIAL_BUFF_SIZE + 2];
  uint16_t dataLen;
  uint8_t startFlag;
} serialIntf_t;


static uint8_t gBoardNum = SENSOR_BOARD_NONE;
static uint16_t gProcessTime = 0;
static uint16_t gFullTime = 0;

static sensorIns_t gSensorInstances[SENSOR_COUNT] = {0};
static sensorReadingSm_t gSensorReadingSm = {0};
static buzzerSm_t gBuzzerSm = {0};
static serialIntf_t gSerialIntf={0};



static PCF8575 gIoExpander(IO_EXPANDER_I2C_ADDR);
static SoftwareSerial gSerial(SERIAL_RX_PIN, SERIAL_TX_PIN);


static void InitDipSwitchs(void) {
  pinMode(DIP_SWITCH_PIN_1, INPUT);
  pinMode(DIP_SWITCH_PIN_2, INPUT);
  pinMode(DIP_SWITCH_PIN_3, INPUT);
  pinMode(DIP_SWITCH_PIN_4, INPUT);
}

static uint8_t SetAllCsState(uint8_t state) {

  if (!gIoExpander.isConnected()) {
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

static uint8_t SetCsState(uint8_t csIo, uint8_t state) {

  if (csIo >= SENSOR_COUNT) {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] invalid csIo");
    return false;
  }
  if (!gIoExpander.isConnected()) {
    Serial.print(__LINE__);
    Serial.println(" :[ERR] PCF8575 disconnected");
    return false;
  }

  gIoExpander.write(csIo, state);
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

  SetCsState(csIo, LOW);
  delayMicroseconds(10);

  v = max6675_ReadSpi();
  v <<= 8;
  v |= max6675_ReadSpi();

  SetCsState(csIo, HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN;
  }

  v >>= 3;
  return v * 0.25;
}

static uint8_t DetermineBoardNumber(void) {
  if (digitalRead(DIP_SWITCH_PIN_1) == LOW) {
    return SENSOR_BOARD_1_TO_16;
  }
  else if (digitalRead(DIP_SWITCH_PIN_2) == LOW) {
    return SENSOR_BOARD_17_TO_32;
  }
  else if (digitalRead(DIP_SWITCH_PIN_3) == LOW) {
    return SENSOR_BOARD_33_TO_48;
  }
  else if (digitalRead(DIP_SWITCH_PIN_4) == LOW) {
    return SENSOR_BOARD_49_TO_64;
  }
  else {
    return SENSOR_BOARD_NONE;
  }
}

static void InitializeSensorInstances(void) {
  uint8_t i = 0, start;

  switch (gBoardNum) {
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

  for (i = 0; i < SENSOR_COUNT; i++) {
    gSensorInstances[i].id = start;
    gSensorInstances[i].csIoNum = i;
    gSensorInstances[i].tempRead = 0.0f;
    gSensorInstances[i].timerState = SENSOR_TIMER_STATE_STOP;
    start++;
  }
}

static void ReadAllSensors(void) {
  uint8_t i;
  for (i = 0; i < SENSOR_COUNT; i++) {
    gSensorInstances[i].tempRead = max6675_ReadTemp(gSensorInstances[i].csIoNum);
    if (gSensorInstances[i].tempRead >= TEMPERATURE_MAX_POINT)
      gSensorInstances[i].tempRead = NAN;
  }
}

static void PrintAllSensorData(void) {
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("TEMP[ ");
    Serial.print(gSensorInstances[i].id);
    Serial.print("] value : ");
    Serial.println(gSensorInstances[i].tempRead);
  }
}

static void PrintSensorInstances(void) {
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("{ id: ");
    Serial.print(gSensorInstances[i].id);
    Serial.print(" ioNum: ");
    Serial.print(gSensorInstances[i].csIoNum);
    Serial.print(" tempRead: ");
    Serial.print(gSensorInstances[i].tempRead);
    Serial.println(" }");
  }
}

static void StartBuzzer(void) {
  gBuzzerSm.state = BUZZER_STATE_START;
}

static void StopBuzzer(void) {
  gBuzzerSm.state = BUZZER_STATE_STOP;
}

static void BuzzerSM(void) {

  switch (gBuzzerSm.state)
  {
    case BUZZER_STATE_START:
      {
        // TODO use macro
        digitalWrite(BUZZER_PIN, HIGH);
        gBuzzerSm.startMs = millis();
        gBuzzerSm.state = BUZZER_STATE_WAIT;
        break;
      }
    case BUZZER_STATE_WAIT:
      {
        gBuzzerSm.deltaMs = millis() - gBuzzerSm.startMs;
        if (gBuzzerSm.deltaMs >= (BUZZER_RUN_TIME * 1000)) {
          // TODO use macro
          digitalWrite(BUZZER_PIN, LOW);
          gBuzzerSm.state = BUZZER_STATE_STOP;
          // TODO log disable
          Serial.print(__func__);
          Serial.print(" : deltaMs: ");
          Serial.println(gBuzzerSm.deltaMs);
        }
        break;
      }
    case BUZZER_STATE_STOP:
      {
        // do nothing idelling state
        break;
      }
    default:
      {
        // TODO stop program
        Serial.print(__LINE__);
        Serial.println(": [ERR] invalid state");
        break;
      }
  }
}


static void SensorReadingSM(uint8_t state) {
  switch (state) {
    case SENSOR_READ_STATE_START: {
        Serial.println("Sensor Reading started");
        gSensorReadingSm.state = SENSOR_READ_STATE_READ;
        break;
      }
    case SENSOR_READ_STATE_READ: {
        ReadAllSensors();
        gSensorReadingSm.startMs = millis();
        PrintAllSensorData();
        gSensorReadingSm.state = SENSOR_READ_STATE_WAIT;
        break;
      }
    case SENSOR_READ_STATE_WAIT: {
        gSensorReadingSm.deltaMs = millis() - gSensorReadingSm.startMs;
        if (gSensorReadingSm.deltaMs >= 250) // TODO use macro
        {
          Serial.print("deltaTime: ");
          Serial.print(gSensorReadingSm.deltaMs);
          Serial.println(" ms");
          gSensorReadingSm.state = SENSOR_READ_STATE_READ;
        }
        break;
      }
    case SENSOR_READ_STATE_STOP: {
        // do nothing idelling state
        break;
      }

    default : {
        // TODO stop program
        Serial.print(__LINE__);
        Serial.println(" :[ERR] invalid state");
        break;
      }
  }
}

static void SensorTimerSM(uint8_t index) {

  if (index >= SENSOR_COUNT - 1) {
    Serial.print(__LINE__);
    Serial.println("invalid input");
  }

  switch (gSensorInstances[index].timerState)
  {

    case SENSOR_TIMER_STATE_START:
      {
        // TODO millis will expires in 49 days.
        gSensorInstances[index].startTimeMs = millis();
        gSensorInstances[index].timerState = SENSOR_TIMER_STATE_WAIT;
        break;
      }
    case SENSOR_TIMER_STATE_WAIT:
      {
        gSensorInstances[index].runTimeMs = millis() - gSensorInstances[index].startTimeMs;



        // checks for half time
        if ( gSensorInstances[index].runTimeMs >= ((gProcessTime / 2) * 1000) &&
             gSensorInstances[index].flags.halfTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_HALF_TIME;
        }
        else if ( gSensorInstances[index].runTimeMs >= (gProcessTime * 1000) &&
                  gSensorInstances[index].flags.fullTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_FULL_TIME;
        }
        else if (gSensorInstances[index].runTimeMs >= ((gProcessTime + ADDITIONAL_TIME) * 1000)  &&
                 gSensorInstances[index].flags.endTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_END_TIME;
        }
        else {

          // TODO hanlde error
          //          if (gSensorInstances[index].runTimeMs > (gProcessTime + ADDITIONAL_TIME)) {
          //            Serial.print(__LINE__);
          //            Serial.println("critical error");
          //          }
          //TODO do nothing
        }

        break;
      }
    case SENSOR_TIMER_STATE_HALF_TIME:
      {
        StartBuzzer();
        // TODO half time readed
        gSensorInstances[index].flags.halfTime = true;
        gSensorInstances[index].timerState = SENSOR_TIMER_STATE_WAIT;
        break;
      }
    case SENSOR_TIMER_STATE_FULL_TIME:
      {
        StartBuzzer();
        // TODO full time reached
        gSensorInstances[index].flags.fullTime = true;
        gSensorInstances[index].timerState = SENSOR_TIMER_STATE_WAIT;
        break;
      }

    case SENSOR_TIMER_STATE_END_TIME:
      {
        StartBuzzer();
        // TODO end time reached
        gSensorInstances[index].flags.endTime = true;
        gSensorInstances[index].timerState = SENSOR_TIMER_STATE_STOP;
        break;
      }

    case SENSOR_TIMER_STATE_STOP:
      {
        // do nothing
        break;
      }

    default : {
        // TODO stop program
        Serial.print(__LINE__);
        Serial.println(" :[ERR] invalid state");
        break;
      }
  }
}

static void SetTemperatureSetPointsToDefault(void) {
  uint8_t i;
  for (i = 0; i < SENSOR_COUNT; i++) {
    if (gSensorInstances[i].tempSet <= 0) {
      gSensorInstances[i].tempSet = TEMPERATURE_SET_POINT_DEFAULT;
    }
  }
}

static void CalculateProcessTime(void) {
  float temp = gSensorInstances[SENSOR_COUNT - 1].tempRead;
  if (temp != NAN && (int)temp < TEMPERATURE_MAX_POINT) {
    gProcessTime = (435 + (325 - temp)) + (60 * ((325 - temp) / 50)); // TODO calculate actual process time
    Serial.print("Process Time: ");
    Serial.print(gProcessTime);
    Serial.println(" sec");
  } else {
    gProcessTime = 0;
  }
}

static void CheckForTemperatureTriggers(void) {
  uint8_t i;
  for (i = 0; i < (SENSOR_COUNT - 1); i++) {
    if (gSensorInstances[i].tempSet <= 0 )
    {
      Serial.print(__LINE__);
      Serial.println(" :[ERR] invalid set temperature");
      break;
    } else if (gSensorInstances[i].tempRead == NAN) {
      Serial.print(__LINE__);
      Serial.println(" :[ERR] invalid temperature");
      break;
    }
    else if ((int)gSensorInstances[i].tempRead >=
             gSensorInstances[i].tempSet)
    {
      // TODO find ways to disable logs
      Serial.print("Temperature Set Point Reached :[ ");
      Serial.print(i);
      Serial.println(" ]");

      // Only start timer if timer not running ie.  SENSOR_TIMER_STATE_STOP
      if (gSensorInstances[i].timerState == SENSOR_TIMER_STATE_STOP)
      {
        gSensorInstances[i].timerState = SENSOR_TIMER_STATE_START;
      }
      else
      {
        Serial.print("[WRN] Timer allready running for sensor id[ ");
        Serial.print(gSensorInstances[i].id);
        Serial.println(" ]");
      }


    }

  }
}

static void ClearSerialData(void) {
  // reset len
  gSerialIntf.dataLen = 0;
  // reset data
  (void)memset(gSerialIntf.dataBuff, '\0', sizeof(gSerialIntf.dataBuff));
}

static uint8_t ParseSerialData(const char *data,uint16_t len){
  return true;
}

static void ProcessSerialData(void) {
  char ch = '\0';
  if (gSerial.available() > 0) {
    
    ch = gSerial.read();
    
    if (gSerialIntf.dataLen >= sizeof(gSerialIntf.dataBuff)) {
      Serial.print(__LINE__);
      Serial.println("[WRN] Buffer Overflow");
      ClearSerialData();
    }
    else if (ch == '\n') {
      gSerialIntf.dataBuff[gSerialIntf.dataLen] = ch;
      gSerialIntf.dataLen++;
      // data ready to parse
      if(ParseSerialData(gSerialIntf.dataBuff,gSerialIntf.dataLen)){
       gSerial.println("Valid");
      }
      
      ClearSerialData();
    } else {
      gSerialIntf.dataBuff[gSerialIntf.dataLen] = ch;
      gSerialIntf.dataLen++;
    }

  }
}


void setup() {

  Wire.begin();

  Serial.begin(9600);
  Serial.println();

  gSerial.begin(9600);
  gSerial.println("Hello soft serail");

  pinMode(BUZZER_PIN, OUTPUT);

  InitDipSwitchs();

  gBoardNum = DetermineBoardNumber();

  while (gBoardNum <= SENSOR_BOARD_NONE
         || gBoardNum >= SENSOR_BOARD_MAX)
  {
    Serial.println(__LINE__);
    Serial.print(" : [ERR] Invalid Board Number: ");
    Serial.println(gBoardNum);
  }

  Serial.print("Board number: ");
  Serial.println(gBoardNum);

  InitializeSensorInstances();

  // TODO read set points from EEPROM

  /*
    If temperature set point not found in EEPROM
    set points to TEMPERATURE_SET_POINT_DEFAULT
  */
  SetTemperatureSetPointsToDefault();

  PrintSensorInstances();


  while (!gIoExpander.begin()) {
    Serial.print(__LINE__);
    Serial.println(" : [ERR] PCF8575 init failed");
  }

  gSerial.begin(SERIAL_BAUD_RATE);

  max6675_Init();

  // start sensor state machine
  gSensorReadingSm.state = SENSOR_READ_STATE_START;

  StopBuzzer();
}

void loop() {

  ProcessSerialData();

  //  // Sensor reading state machine
  //  SensorReadingSM(gSensorReadingSm.state);
  //
  //  // Calculate time value
  //  CalculateProcessTime();
  //
  //  // Check for trigger
  //  CheckForTemperatureTriggers();
  //
  //  for (uint8_t i = 0; i < (SENSOR_COUNT - 1); i++) {
  //    SensorTimerSM(i);
  //  }
  //
  //  BuzzerSM();

  //  startTime=millis();
  //  ReadAllSensors();
  //  deltaTime=millis()-startTime;
  //  Serial.print("Time : ");
  //  Serial.print(deltaTime);
  //  Serial.println(" ms");
  //  PrintAllSensorData();
}
