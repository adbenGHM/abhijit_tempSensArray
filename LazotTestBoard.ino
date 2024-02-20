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



/*-------------------------------------------*/




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


void setup() {

  Serial.begin(9600);

  Wire.begin();
  
  if(!gIoExpander.begin()){
    Serial.println("PCF8575 init failed");
  }
  Serial.println("PCF8575 initialized");
  
  InitTemperature();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp;

  uint8_t i;
  for(i=0;i<16;i++){
    gSensorToRead=i;
    temp=ReadTemperatureInDegC();
    Serial.print("Reading sensor[ ");
    Serial.print(i);
    Serial.print("] value : ");
    Serial.print(temp);
    Serial.println(" degC");
    delay(500);
  }
   Serial.println("\r\n");
  gSensorToRead=0;
}
