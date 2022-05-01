/*=============================================================================
File Name           : 
Project File Number : v 1.00b
Project Name        : 
Author              : HS & MAH
Start Date          : 
Chip                : 

Copyright(c) 2022, All Rights Reserved.
-------------------------------------------------------------------------------
Description: 
-----------------------------------------------------------------------------*/
/***************************   Include Files   *******************************/
#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
//            SDA   SCL
TwoWire Wire1(PB7,  PB6);
TwoWire Wire2(PB11, PB10);
/***************************   Defines   *************************************/
#define LED                       PC13 // the number of the ALIVELED pin
#define SDP810_I2C_ADDR           0x25 // I2C Address for SDP8xx device
#define FILTER_CONST              0.90f //
/***************************   Flags and bit *********************************/
/***************************   sbit ******************************************/
/***************************   Macros   **************************************/
/***************************   Data Types   **********************************/
/***************************   Local Variables   *****************************/
float               FilteredPressure_1   = 0.0;
float               FilteredTemperatur_1 = 0.0;
float               FilteredPressure_2   = 0.0;
float               FilteredTemperatur_2 = 0.0;
//
u_int64_t       previousMillis     = 0; // will store last time LED was updated
u_int64_t       currentMillis;


/*****************************************************************************/

/***************************  Enum  ******************************************/
typedef enum 
{ 
    IDLE_STATE = 0, 
    SEND_I2C_READ_SENSOR_STATE = 1, 
    READ_I2C_SENSOR_STATE = 2,
    SDP_TO_FLOW_STATE = 3,
    PRINT_SDP_VALUE_STATE = 4
} sensorState;

static sensorState currentSdp810State = IDLE_STATE;
static sensorState prevState = IDLE_STATE;
/***************************   Constants   ***********************************/


/***************************   Global Variables   ****************************/

/***************************   Function Prototypes   *************************/
void setup(void);
void SetupStm32(void);
void Loop(void);
void GetSdPressure_1(void);
void GetSdPressure_2(void);
void displayState();
void ResetSensor_1(void);
/******************************************************************************
Function name : void setup()
         Type : PRIVATE
Description   : Run after start/reset
Notes :
******************************************************************************/
//Serial1 setup
//                      RX    TX
//HardwareSerial1 Serial1(PA10, PA9);
void setup() {
    SetupStm32();
    ResetSensor_1();
    delay(100);   
}
/******************************************************************************
Function name : void Loop()
         Type : PRIVATE
Description   :
Notes :
******************************************************************************/
void loop() 
{
    GetSdPressure_1();
    GetSdPressure_2();

} // END loop
/******************************************************************************
Function name : 
         Type :
Description   : 
Notes :
******************************************************************************/
/******************************************************************************
Function name : GetSdPressure_1(void)
         Type : PRIVATE
Description   : Read Sensor value use FSM
Notes :
******************************************************************************/
//
void GetSdPressure_1(void){
/***************************   Local Variables   *****************************/
volatile static float      difPressure_1        = 0.0;
volatile static float      temperatur_1         = 0.0;
static   u_int32_t         interval             = millis();
static   u_int32_t         printInterval        = millis();
static   u_int8_t          n                    = 0;
volatile int16_t           pressure_sint;
volatile int16_t           temperatur_sint;
volatile static            u_int8_t data[8];

    switch (currentSdp810State) 
    {
        case IDLE_STATE:{
            // Do nothing here for now
            // Move to next state
            currentSdp810State = SEND_I2C_READ_SENSOR_STATE;
        }  
            break;
        // 
        case SEND_I2C_READ_SENSOR_STATE:{ // ca 280 microsek.
          if (millis() - interval >= 10) {
            // step : instruct sensor command
            Wire1.beginTransmission(SDP810_I2C_ADDR); // transmit to device #37 (0x25)
            //
            Wire1.write(byte(0x36));      // msb
            Wire1.write(byte(0x15));      // lsb command sensor Differential pressure Average till read (0x3615)
            Wire1.endTransmission();      // stop transmitting
            interval = millis();
            // Move to next state
            currentSdp810State = READ_I2C_SENSOR_STATE;            
          }  
        }    
            break;
        //
        case READ_I2C_SENSOR_STATE:
            // step : request reading from sensor
            Wire1.requestFrom(SDP810_I2C_ADDR, 8);    // request 2 + crc bytes from slave device #37 (0x25)
            for (n=0;n<8;n++)
            {
                data[n] = Wire1.read();          // receive high byte (overwrites previous reading)
            }
            pressure_sint = (int16_t)(data[0]*256+data[1]);
            difPressure_1 = (float)(pressure_sint) / data[7]; // Scale Factor for 125pa type
            temperatur_sint = (int16_t)(data[3]*256+data[4]);
            temperatur_1 = (float)(temperatur_sint) / data[7]; // Scale Factor "data byte 7"
            FilteredPressure_1 = FILTER_CONST*FilteredPressure_1 + difPressure_1*(1-FILTER_CONST);
            FilteredTemperatur_1 = FILTER_CONST*FilteredTemperatur_1 + temperatur_1*(1-FILTER_CONST);
            currentSdp810State = SDP_TO_FLOW_STATE;
            break;
        //    
        case SDP_TO_FLOW_STATE:{
            // Move to next state 
            currentSdp810State = PRINT_SDP_VALUE_STATE;
          }
            break;
        //    
        case PRINT_SDP_VALUE_STATE:{
            // Serial1.print(" - ");
        if (millis() - printInterval >= 1000) {
            Serial1.print("DP_1:  ");
            Serial1.print(FilteredPressure_1);
            Serial1.print(" T_1:  ");
            Serial1.print(FilteredTemperatur_1);
            printInterval = millis(); 
//            digitalWrite(LED, !digitalRead(LED));  
          }
            // Move to next state
            currentSdp810State = IDLE_STATE;
        }
            break;
        //    
        default:
            currentSdp810State = IDLE_STATE;
            break;
  }
}
/******************************************************************************
Function name : GetSdPressure_2(void)
         Type : PRIVATE
Description   : Read Sensor value use FSM
Notes :
******************************************************************************/
//
void GetSdPressure_2(void){
/***************************   Local Variables   *****************************/
volatile static float      difPressure_2        = 0.0;
volatile static float      temperatur_2         = 0.0;
static   u_int32_t         interval             = millis();
static   u_int32_t         printInterval        = millis();
static   u_int8_t          n                    = 0;
volatile int16_t           pressure_sint;
volatile int16_t           temperatur_sint;
volatile static            u_int8_t data[8];

    switch (currentSdp810State) 
    {
        case IDLE_STATE:{
            // Do nothing here for now
            // Move to next state
            currentSdp810State = SEND_I2C_READ_SENSOR_STATE;
        }  
            break;
        // 
        case SEND_I2C_READ_SENSOR_STATE:{ // ca 280 microsek.
          if (millis() - interval >= 10) {
            // step : instruct sensor command
            Wire2.beginTransmission(SDP810_I2C_ADDR); // transmit to device #37 (0x25)
            //
            Wire2.write(byte(0x36));      // msb
            Wire2.write(byte(0x15));      // lsb command sensor Differential pressure Average till read (0x3615)
            Wire2.endTransmission();      // stop transmitting
            interval = millis();
            // Move to next state
            currentSdp810State = READ_I2C_SENSOR_STATE;            
          }  
        }    
            break;
        //
        case READ_I2C_SENSOR_STATE:
            // step : request reading from sensor
            Wire2.requestFrom(SDP810_I2C_ADDR, 8);    // request 2 + crc bytes from slave device #37 (0x25)
            for (n=0;n<8;n++)
            {
                data[n] = Wire2.read();          // receive high byte (overwrites previous reading)
            }
            pressure_sint = (int16_t)(data[0]*256+data[1]);
            difPressure_2 = (float)(pressure_sint) / data[7]; // Scale Factor for 125pa type
            temperatur_sint = (int16_t)(data[3]*256+data[4]);
            temperatur_2 = (float)(temperatur_sint) / data[7]; // Scale Factor "data byte 7"
            FilteredPressure_2 = FILTER_CONST*FilteredPressure_2 + difPressure_2*(1-FILTER_CONST);
            FilteredTemperatur_2 = FILTER_CONST*FilteredTemperatur_2 + temperatur_2*(1-FILTER_CONST);
            currentSdp810State = SDP_TO_FLOW_STATE;
            break;
        //    
        case SDP_TO_FLOW_STATE:{
            // Move to next state 
            currentSdp810State = PRINT_SDP_VALUE_STATE;
          }
            break;
        //    
        case PRINT_SDP_VALUE_STATE:{
            // Serial1.print(" - ");
        if (millis() - printInterval >= 1000) {
            Serial1.print("  ");
            Serial1.print("DP_2:  ");
            Serial1.print(FilteredPressure_2);
            Serial1.print(" T_2:  ");
            Serial1.println(FilteredTemperatur_2);
            printInterval = millis(); 
            digitalWrite(LED, !digitalRead(LED));                        
        }
            // Move to next state
            currentSdp810State = IDLE_STATE;
        }
            break;
        //    
        default:
            currentSdp810State = IDLE_STATE;
            break;
  }
}
/******************************************************************************
Function name : void displayState(String currentSdp810State)
         Type : PRIVATE
Description   : Print out the state
Notes :
******************************************************************************/
void displayState() {
    if (currentSdp810State != prevState) {
        Serial1.println(currentSdp810State);
        prevState = currentSdp810State;
    }
}
/******************************************************************************
Function name : void SetupStm32(void)
         Type :
Description   : 
Notes :
******************************************************************************/
void SetupStm32(void)
 {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); // set to known state
    //Wire1.setClock(400000);  // Set I2C clock to 400 kHz
    //Wire2.setClock(400000);  // Set I2C clock to 400 kHz
    Serial1.begin(115200);
    Wire1.begin();
    Wire2.begin();
    delay(100);
 }
 /******************************************************************************
Function name : void ResetSensor_1()
         Type :
Description   : Send reset to SDP810 adr 0x25
Notes :
******************************************************************************/
void ResetSensor_1()
{
   // step : instruct sensor command
   Wire1.beginTransmission(SDP810_I2C_ADDR); // transmit to device #37 (0x25)
   Wire1.write(byte(0x00)); // msb
   Wire1.write(byte(0x06)); // lsb command sensor Soft Reset (0x0006)
   Wire1.endTransmission(); // stop transmitting
   delay(50);
}