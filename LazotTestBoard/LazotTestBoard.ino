#include <string.h>
#include <PCF8575.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "ArduinoJson.h"


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
#define SERIAL_BUFF_SIZE (200+2)

// SOLINOID
#define SOLENOID_PIN (9)

// MODE
#define DEFAULT_MODE (1)

// EEPROM
#define EEPROM_START_ADDR (0x02)
#define EEPROM_DATA_OFFSET (0x02)


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

#define TIME_FOR_MODE_1 (1*60)
#define TIME_FOR_MODE_2 (2*60)
#define TIME_FOR_MODE_3 (3*60)
#define TIME_FOR_MODE_4 (4*60)

typedef enum {
  SENSOR_MODE_NONE = 0,
  SENSOR_MODE_1 = 1,
  SENSOR_MODE_2 = 2,
  SENSOR_MODE_3 = 3,
  SENSOR_MODE_4 = 4,
  SENSOR_MODE_MAX,
} sensorMode_t;


typedef enum {
  CMD_TYPE_NONE = 0,
  CMD_TYPE_SET_TEMP = 1,
  CMD_TYPE_GET_TEMP = 2,
  CMD_TYPE_SET_MODE = 3,
  CMD_TYPE_MAX,
} cmdType_t;




typedef struct {
  uint8_t id;
  uint8_t csIoNum;
  uint8_t mode;
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
  uint8_t cr;
} serialIntf_t;

typedef struct {
  uint8_t type;
  union {
    struct {
      uint8_t id;
      uint16_t val;
    } setTemp;

    struct {
      uint8_t id;
    } getTemp;


    struct {
      uint8_t id;
      uint8_t mode;
    } setMode;

  } params;

} cmdInst_t;


//static const char *TRUE_VALUE_STR = "true";
//static const char *FALSE_VALUE_STR = "false";
//static const char *GET_CMD_FORMAT = "{\"temp\":%u,\"temp_set\":%u,\"time\":%u,\"half_time\":%s,\"full_time\":%s,\"end_time\":%s,\"mode\":%u}";
typedef struct {
  uint8_t mode;
  uint16_t  tempSet;
} romStruct_t;

static const uint8_t EEPROM_ADDRESS_TABLE[] = {
  0x00,
  0x03,
  0x06,
  0x09,
  0x0c,
  0x0f,
  0x12,
  0x15,
  0x18,
  0x1b,
  0x1e,
  0x21,
  0x24,
  0x27,
  0x2a,
};

static uint8_t gBoardNum = SENSOR_BOARD_NONE;
static uint8_t gMode = DEFAULT_MODE;
static uint32_t gProcessTime = 0;
static uint32_t gFullTimeMs = 0;

static sensorIns_t gSensorInstances[SENSOR_COUNT] = {0};
static sensorReadingSm_t gSensorReadingSm = {0};
static buzzerSm_t gBuzzerSm = {0};
static serialIntf_t gSerialIntf = {0};
static cmdInst_t gCmdInst = {0};



static PCF8575 gIoExpander(IO_EXPANDER_I2C_ADDR);
static SoftwareSerial gSerial(SERIAL_RX_PIN, SERIAL_TX_PIN);

static void ErrorHandler(int line, const char *reason) {
  while (1) {
    Serial.print(__LINE__);
    Serial.print(" :[ERR]");
    Serial.println(reason);
    delay(1000);
  }
}


static void InitDipSwitchs(void) {
  pinMode(DIP_SWITCH_PIN_1, INPUT);
  pinMode(DIP_SWITCH_PIN_2, INPUT);
  pinMode(DIP_SWITCH_PIN_3, INPUT);
  pinMode(DIP_SWITCH_PIN_4, INPUT);
}

static uint8_t SetAllCsState(uint8_t state) {

  if (!gIoExpander.isConnected()) {
    Serial.print(__LINE__);
    Serial.println(F(" :[ERR] PCF8575 disconnected"));
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
    Serial.println(F(" :[ERR] invalid csIo"));
    return false;
  }
  if (!gIoExpander.isConnected()) {
    Serial.print(__LINE__);
    Serial.println(F(" :[ERR] PCF8575 disconnected"));
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
    return 0.0f;
  }

  v >>= 3;
  return v * 0.25;
}

static uint8_t IsIdValid(uint8_t id) {
  uint8_t ret = true;

  switch (gBoardNum) {
    case SENSOR_BOARD_1_TO_16:
      ret = (id < 1 || id >= 16 ) ? false : true;
      break;
    case SENSOR_BOARD_17_TO_32:
      ret = (id < 17 || id >= 32 ) ? false : true;
      break;
    case SENSOR_BOARD_33_TO_48:
      ret = (id < 33 || id >= 48 ) ? false : true;
      break;
    case SENSOR_BOARD_49_TO_64:
      ret = (id < 49 || id >= 64 ) ? false : true;
      break;
    default:
      ret = false;
      break;
  }
  return ret;
}

static uint8_t IsModeValid(uint8_t mode) {
  if (mode <= 0 && mode >= 5)
    return false;
  else
    return true;
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

static void ReadSetTempsFromRom(void) {
  uint8_t i;
  for (i = 0; i < (SENSOR_COUNT - 1); i++) {
    romStruct_t data = {0};
    EEPROM.get(EEPROM_ADDRESS_TABLE[i], data);
    gSensorInstances[i].mode = data.mode;
    gSensorInstances[i].tempSet = data.tempSet;
  }
}

static uint8_t WriteSetTempToRom(uint8_t id, uint16_t val) {
  if (val <= 0 || val >= TEMPERATURE_MAX_POINT)
    return false;
  uint8_t i, found = false;

  for ( i = 0; i < (SENSOR_COUNT - 1); i++) {
    if (gSensorInstances[i].id == id) {
      found = true;
      break;
    }
  }

  if (found) {
    romStruct_t data = {0};
    EEPROM.get(EEPROM_ADDRESS_TABLE[i], data);
    data.tempSet = val;
    EEPROM.put(EEPROM_ADDRESS_TABLE[i], data);
    gSensorInstances[i].tempSet = val;
    return true;
  } else
    return false;
}

static uint8_t WriteModeToRom(uint8_t id, uint8_t mode) {
  if (mode <= SENSOR_MODE_NONE || mode >= SENSOR_MODE_MAX)
    return false;
  uint8_t i, found = false;

  for ( i = 0; i < (SENSOR_COUNT - 1); i++) {
    if (gSensorInstances[i].id == id) {
      found = true;
      break;
    }
  }

  if (found) {
    romStruct_t data = {0};
    EEPROM.get(EEPROM_ADDRESS_TABLE[i], data);
    data.mode = mode;
    EEPROM.put(EEPROM_ADDRESS_TABLE[i], data);
    gSensorInstances[i].mode = mode;
    return true;
  } else
    return false;
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
    gSensorInstances[i].tempSet = TEMPERATURE_SET_POINT_DEFAULT;
    gSensorInstances[i].timerState = SENSOR_TIMER_STATE_STOP;
    gSensorInstances[i].startTimeMs = 0;
    gSensorInstances[i].runTimeMs = 0;
    gSensorInstances[i].flags.halfTime = false;
    gSensorInstances[i].flags.fullTime = false;
    gSensorInstances[i].flags.endTime = false;
    start++;
  }
}

void SetSettingsToDefault(void) {
  uint8_t i;
  for (i = 0; i < (SENSOR_COUNT - 1); i++) {
    if (gSensorInstances[i].mode <= SENSOR_MODE_NONE
        || gSensorInstances[i].mode >= SENSOR_MODE_MAX) {
      gSensorInstances[i].mode = SENSOR_MODE_1;
    }
    if (gSensorInstances[i].tempSet <= 0
        || gSensorInstances[i].tempSet >= TEMPERATURE_MAX_POINT) {
      gSensorInstances[i].tempSet = TEMPERATURE_SET_POINT_DEFAULT;
    }
  }
}

static void ReadAllSensors(void) {
  uint8_t i;
  for (i = 0; i < SENSOR_COUNT; i++) {
    gSensorInstances[i].tempRead = max6675_ReadTemp(gSensorInstances[i].csIoNum);
    if (gSensorInstances[i].tempRead >= TEMPERATURE_MAX_POINT )
      gSensorInstances[i].tempRead = 0.0f;
  }
}

static void PrintAllSensorData(void) {
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("TEMP[ "));
    Serial.print(gSensorInstances[i].id);
    Serial.print(F("] value : "));
    Serial.println(gSensorInstances[i].tempRead);
  }
}

static void PrintSensorInstances(void) {
  uint8_t i;
  Serial.println();
  for (i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("{ id: "));
    Serial.print(gSensorInstances[i].id);
    Serial.print(F(" ioNum: "));
    Serial.print(gSensorInstances[i].csIoNum);
    Serial.print(F(" tempSet: "));
    Serial.print(gSensorInstances[i].tempSet);
    Serial.print(F(" mode: "));
    Serial.print(gSensorInstances[i].mode);
    Serial.println(F(" }"));
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
        digitalWrite(BUZZER_PIN, HIGH);
        gBuzzerSm.startMs = millis();
        gBuzzerSm.state = BUZZER_STATE_WAIT;
        break;
      }
    case BUZZER_STATE_WAIT:
      {
        gBuzzerSm.deltaMs = millis() - gBuzzerSm.startMs;
        if (gBuzzerSm.deltaMs >= (BUZZER_RUN_TIME * 1000)) {
          digitalWrite(BUZZER_PIN, LOW);
          gBuzzerSm.state = BUZZER_STATE_STOP;

          // TODO diable logs
          //          Serial.print(__func__);
          //          Serial.print(" : deltaMs: ");
          //          Serial.println(gBuzzerSm.deltaMs);
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
        ErrorHandler(__LINE__, "invalid state");
        break;
      }
  }
}


static void SensorReadingSM(uint8_t state) {

  switch (state) {
    case SENSOR_READ_STATE_START: {
        gSensorReadingSm.state = SENSOR_READ_STATE_READ;

        // TODO disbale logs
        //        Serial.println("Sensor Reading started");
        break;
      }
    case SENSOR_READ_STATE_READ: {

        ReadAllSensors();
        gSensorReadingSm.startMs = millis();
        gSensorReadingSm.state = SENSOR_READ_STATE_WAIT;

        // TODO disbale logs
        //        PrintAllSensorData();
        break;
      }
    case SENSOR_READ_STATE_WAIT: {
        gSensorReadingSm.deltaMs = millis() - gSensorReadingSm.startMs;
        if (gSensorReadingSm.deltaMs >= 250) // TODO use macro
        {
          gSensorReadingSm.state = SENSOR_READ_STATE_READ;

          // TODO disable logs
          //          Serial.print("deltaTime: ");
          //          Serial.print(gSensorReadingSm.deltaMs);
          //          Serial.println(" ms");

        }
        break;
      }
    case SENSOR_READ_STATE_STOP: {
        // do nothing idelling state
        break;
      }

    default : {
        ErrorHandler(__LINE__, "invalid state");
        break;
      }
  }
}

static void SensorTimerSM(uint8_t index) {

  if (index >= SENSOR_COUNT - 1)
    ErrorHandler(__LINE__, "invalid input");

  switch (gSensorInstances[index].timerState)
  {
    case SENSOR_TIMER_STATE_START:
      {
        gSensorInstances[index].startTimeMs = millis();
        gSensorInstances[index].timerState = SENSOR_TIMER_STATE_WAIT;
        break;
      }
    case SENSOR_TIMER_STATE_WAIT:
      {
        gSensorInstances[index].runTimeMs = millis() - gSensorInstances[index].startTimeMs;
        gFullTimeMs = (gProcessTime * 1000);

        // checks for half time
        if ( gSensorInstances[index].runTimeMs >= gFullTimeMs / 2 &&
             gSensorInstances[index].flags.halfTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_HALF_TIME;
        }
        else if ( gSensorInstances[index].runTimeMs >= gFullTimeMs &&
                  gSensorInstances[index].flags.fullTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_FULL_TIME;
        }
        else if (gSensorInstances[index].runTimeMs >= (gFullTimeMs + (ADDITIONAL_TIME * 1000)) &&
                 gSensorInstances[index].flags.endTime == false)
        {
          gSensorInstances[index].timerState = SENSOR_TIMER_STATE_END_TIME;
        }
        else {
          // TODO is it valid
          if (gSensorInstances[index].runTimeMs > (gFullTimeMs + (ADDITIONAL_TIME * 1000)) &&
              gSensorInstances[index].flags.endTime == true) {
            ErrorHandler(__LINE__, "critical error");
          }
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
        Serial.println(F(" :[ERR] invalid state"));
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
    //    Serial.print(F("Process Time: "));
    //    Serial.print(gProcessTime);
    //    Serial.println(F(" sec"));
  } else {
    gProcessTime = 0;
  }
}

static void StartTimer(uint8_t index) {
  // start timer only when timer is not running
  // ie.SENSOR_TIMER_STATE_STOP
  if (gSensorInstances[index].timerState == SENSOR_TIMER_STATE_STOP)
    gSensorInstances[index].timerState = SENSOR_TIMER_STATE_START;

}

static void CheckForTemperatureTriggers(void) {
  uint8_t i;
  for (i = 0; i < (SENSOR_COUNT - 1); i++) {

    if (gSensorInstances[i].tempSet <= 0 || gSensorInstances[i].tempSet >= TEMPERATURE_MAX_POINT)
    {
      ErrorHandler(__LINE__, "invalid temperature setting");
      break;
    } else if (gSensorInstances[i].tempRead == NAN) {

      // TODO disable log
      Serial.print(gSensorInstances[i].id);
      Serial.println(F("prob disconnected"));

      break;
    }
    else if ((uint16_t)gSensorInstances[i].tempRead >=
             gSensorInstances[i].tempSet)
    {
      // TODO disable log
      Serial.print(gSensorInstances[i].id);
      Serial.println(F(" Probe Set point reached"));
      StartTimer(i);
    }

  }
}



static void CreateGetCmdResponce(uint8_t id) {

  uint8_t i = 0;
  for (i = 0; i < SENSOR_COUNT; i++) {
    if (gSensorInstances[i].id == id) {
      break;
    }
  }
  memset(gSerialIntf.dataBuff, 0, sizeof(gSerialIntf.dataBuff));
  JsonDocument doc;
  doc["temp"] = gSensorInstances[i].tempRead;
  doc["temp_set"] = (uint16_t)gSensorInstances[i].tempSet;
  doc["time_remain"] = (uint16_t)(gProcessTime - gSensorInstances[i].runTimeMs);
  doc["half_time"] = (bool)gSensorInstances[i].flags.halfTime;
  doc["full_time"] = (bool)gSensorInstances[i].flags.fullTime;
  doc["end_time"] = (bool)gSensorInstances[i].flags.endTime;
  doc["mode"] = (uint8_t)1;

  serializeJson(doc, gSerialIntf.dataBuff, sizeof(gSerialIntf.dataBuff));

}

static void ParseSerialData(const char *data) {

  gCmdInst.type = CMD_TYPE_NONE;

  if (data == NULL) {
    Serial.print(__LINE__);
    Serial.println(F("nullptr"));
    return ;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    Serial.print(__LINE__);
    Serial.println(F(" :[WRN]invalid json"));
    return ;
  }

  const char *cmd = doc["cmd"];

  if (strcmp(cmd, "set") == 0) {
    gCmdInst.params.setTemp.id = doc["id"].as<uint8_t>();
    gCmdInst.params.setTemp.val = (uint16_t)doc["val"].as<float>();
    gCmdInst.type = IsIdValid(gCmdInst.params.setTemp.id) ? CMD_TYPE_SET_TEMP : CMD_TYPE_NONE;

  } else if (strcmp(cmd, "get") == 0) {

    gCmdInst.params.getTemp.id = doc["id"].as<uint8_t>();
    gCmdInst.type = IsIdValid(gCmdInst.params.getTemp.id) ? CMD_TYPE_GET_TEMP : CMD_TYPE_NONE;

  } else if (strcmp(cmd, "mode") == 0) {

    gCmdInst.params.setMode.id = doc["id"].as<uint8_t>();
    gCmdInst.params.setMode.mode = doc["mode"].as<uint8_t>();
    gCmdInst.type = IsIdValid(gCmdInst.params.setMode.id) ? CMD_TYPE_SET_MODE : CMD_TYPE_NONE;
    gCmdInst.type = IsModeValid(gCmdInst.params.setMode.mode) ? CMD_TYPE_SET_MODE : CMD_TYPE_NONE;


  } else {
    gCmdInst.type = CMD_TYPE_NONE;
  }
}

static void ClearSerialData(void) {
  gSerialIntf.cr = false;
  // reset len
  gSerialIntf.dataLen = 0;
  // reset data
  (void)memset(gSerialIntf.dataBuff, '\0', sizeof(gSerialIntf.dataBuff));
}

static void ProcessSerialData(void) {
  char ch = '\0';
  if (gSerial.available() > 0) {
    ch = gSerial.read();
    if (gSerialIntf.dataLen >= sizeof(gSerialIntf.dataBuff)) {
      Serial.print(__LINE__);
      Serial.println(F("[WRN] Buffer Overflow"));
      ClearSerialData();
    }
    else if (ch == '\r') {
      gSerialIntf.cr = true;
    }
    else if (ch == '\n') {
      if (gSerialIntf.cr) {
        gSerialIntf.dataBuff[gSerialIntf.dataLen] = '\0';
        gSerialIntf.dataLen++;

        // TODO echo
        //        gSerial.print(gSerialIntf.dataBuff);


        // data ready to parse
        ParseSerialData(gSerialIntf.dataBuff);

        switch (gCmdInst.type) {
          case CMD_TYPE_SET_TEMP: {
              if (WriteSetTempToRom(gCmdInst.params.setTemp.id, gCmdInst.params.setTemp.val))
                gSerial.println(F("ok\r"));
              else
                gSerial.println(F("fail\r"));
              break;
            }
          case CMD_TYPE_GET_TEMP: {
              CreateGetCmdResponce(gCmdInst.params.getTemp.id);
              gSerial.println(gSerialIntf.dataBuff);
              break;
            }
          case CMD_TYPE_SET_MODE: {
              if (WriteModeToRom(gCmdInst.params.setMode.id, gCmdInst.params.setMode.mode))
                gSerial.println(F("ok\r"));
              else
                gSerial.println(F("fail\r"));
              break;
            }
          default:
            break;
        }

      }
      ClearSerialData();
    } else {
      gSerialIntf.dataBuff[gSerialIntf.dataLen] = ch;
      gSerialIntf.dataLen++;
    }

  }
}

static  uint8_t IsBoardNumValid(uint8_t boardNum) {
  return (gBoardNum <= SENSOR_BOARD_NONE
          || gBoardNum >= SENSOR_BOARD_MAX) ? false : true;
}


void setup() {

  Wire.begin();

  Serial.begin(9600);
  Serial.println();

  gSerial.begin(9600);
  gSerial.println(F("Hello soft serail"));

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  InitDipSwitchs();

  gBoardNum = DetermineBoardNumber();

  if (!IsBoardNumValid) {
    ErrorHandler(__LINE__, "invalid board number");
  }

  Serial.print(F("Board number: "));
  Serial.println(gBoardNum);

  InitializeSensorInstances();

  ReadSetTempsFromRom();

  SetSettingsToDefault();

  PrintSensorInstances();

  if (!gIoExpander.begin()) {
    ErrorHandler(__LINE__, "critical error");
  }

  gSerial.begin(SERIAL_BAUD_RATE);

  max6675_Init();

  // start sensor state machine
  gSensorReadingSm.state = SENSOR_READ_STATE_START;

  StopBuzzer();
}

void loop() {

  ProcessSerialData();

  // Sensor reading state machine
  SensorReadingSM(gSensorReadingSm.state);

  // Calculate time value
  CalculateProcessTime();

  // Check for trigger
  CheckForTemperatureTriggers();

  for (uint8_t i = 0; i < (SENSOR_COUNT - 1); i++) {
    SensorTimerSM(i);
  }

  BuzzerSM();

  //  startTime=millis();
  //  ReadAllSensors();
  //  deltaTime=millis()-startTime;
  //  Serial.print("Time : ");
  //  Serial.print(deltaTime);
  //  Serial.println(" ms");
  //  PrintAllSensorData();
}
