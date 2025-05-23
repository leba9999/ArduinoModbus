/*
 * Modbus Arduino Server
 *  
 * https://www.arduino.cc/reference/en/libraries/arduinomodbus/
 *
 * Author: Leevi Koskinen
 * Date: 2023-12-22
 * Version: 1.0.0
 */

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
// Communication
#define BAUD_RATE 9600 // Arduono COM spreed
// Pin registers
#define CONF_START_REG 0x00 // Where configuration registers starts
#define CONF_COUNT 3 // Number of configuration registers
#define DIGI_START_REG 0x03  // Where Digital IO registers starts
#define DIGI_COUNT 18 // Number of Digital IO registers
#define ANALOG_START_REG 0x15 // Where Analog input registers starts
#define ANALOG_COUNT 2 // Number of Analog input registers
/*
  Helper registers only holds values we don't use them any way in arduino code.
  Idea is to hold temporary values if there is need for empty registers
*/
#define HELPER_START_REG 0x17 // Where Helper registers starts
#define HELPER_COUNT 5 // Number of Helper registers
// Reset
#define RESET_START_REG 0x1C // Reset hodling register
#define RESET_COUNT 1 // Number of Reset registers

// Total number of registers
#define TOTAL_REGISTER_COUNT CONF_COUNT+DIGI_COUNT+ANALOG_COUNT+HELPER_COUNT+RESET_COUNT

void setup() {
  // Normal arduino serial communication
  Serial.begin(BAUD_RATE);
  // start the Modbus RTU server, with (slave) id 240
  if (!ModbusRTUServer.begin(240, BAUD_RATE)) {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }
  // Create Modbus Holding registers
  ModbusRTUServer.configureHoldingRegisters(CONF_START_REG, TOTAL_REGISTER_COUNT);
  // Initialize Digital Pin holding registers
  for (int i = DIGI_START_REG; i < DIGI_START_REG + DIGI_COUNT; i++) {
    ModbusRTUServer.holdingRegisterWrite(i, HIGH);
  }
  // Initialize Helper holding registers
  for (int i = HELPER_START_REG; i < HELPER_START_REG + HELPER_COUNT; i++) {
    ModbusRTUServer.holdingRegisterWrite(i, LOW);
  }
  ModbusRTUServer.holdingRegisterWrite(0x0E, LOW); // Set D13 pin holding register to low
  ModbusRTUServer.holdingRegisterWrite(0x1C, LOW); // Initialize reset holding register
  // Initialize holding registers for Arduino registers
  ModbusRTUServer.holdingRegisterWrite(0x00, 0B00000000);
  ModbusRTUServer.holdingRegisterWrite(0x01, 0B00100000); // Sets D13 (LED_BUILTIN) defaults as output and rest as inputs 
  ModbusRTUServer.holdingRegisterWrite(0x02, 0B00000000);
  DDRD = ModbusRTUServer.holdingRegisterRead(0x00);
  DDRB = ModbusRTUServer.holdingRegisterRead(0x01);
  DDRC = ModbusRTUServer.holdingRegisterRead(0x02);
  
  updateArduinoPins();

  Serial.println("Arduino Modbus RTU Server 1.0.0");
}
/**
* Checks is there difference in Arduino registers and Modbus holding registers
* @return true - if there is any changes in the registers
* @return false - if there is not changes in registers
*/
bool checkArduinoRegConf(){
  if(DDRD != ModbusRTUServer.holdingRegisterRead(0x00)){
    return true;
  }
  if(DDRB != ModbusRTUServer.holdingRegisterRead(0x01)){
    return true;
  }
  if(DDRC != ModbusRTUServer.holdingRegisterRead(0x02)){
    return true;
  }
  return false;
}
/**
* Sets Arduino registers same as Modbus holding registers
* Registers which changes updates PORTx bit to 0
* Example if register DDRD changes from default 0B00100000 to 0B00000000 so D13 is now input
* and PORTD is left 0B00100000 it will update it self to 0B00000000
*/
void setArduinoRegs(){
  uint8_t P_DDRD = DDRD; // previous DDRD register state
  uint8_t P_DDRB = DDRB; // previous DDRB register state
  uint8_t P_DDRC = DDRC; // previous DDRC register state
  int register0 = ModbusRTUServer.holdingRegisterRead(0x00);
  int register1 = ModbusRTUServer.holdingRegisterRead(0x01);
  int register2 = ModbusRTUServer.holdingRegisterRead(0x02);
  // Check is register values allowed
  if(register0 >= 0 && register0 <= 255){
    DDRD = register0;
    updatePortOnChange(DDRD, P_DDRD, PORTD);
  }
  if(register1 >= 0 && register1 <= 255){
    DDRB = register1;
    updatePortOnChange(DDRB, P_DDRB, PORTB);
  }
  if(register2 >= 0 && register2 <= 255){
    DDRC = register2;
    updatePortOnChange(DDRC, P_DDRC, PORTC);
  }
}

void updateArduinoPins(){
  // Update arduino pins according to changed holding registers
    for (int i = DIGI_START_REG; i < DIGI_START_REG + DIGI_COUNT; i++) {
      int pin = i-1;
      // Read the current value of the holding register
      int registerValue = ModbusRTUServer.holdingRegisterRead(i);
      if(registerValue >= 0 && registerValue <=1){
        // Set the corresponding pin based on the register value
        digitalWrite(pin, registerValue);
      }
    }
}

/**
* Resets PORTx values to 0 if DDRx has changed. So registers should not get stuck
*/
void updatePortOnChange(uint8_t currentDDR, uint8_t previousDDR, volatile uint8_t &PORT){
  // Create a mask to represent the input state
  uint8_t inputMask = previousDDR ^ currentDDR;  // XOR to identify changes
  // Check if any of the bits in DDR have changed to input
  if ((currentDDR & inputMask) != currentDDR) {
    // If any of the bits are now input, update the corresponding bits in PORT accordingly
    PORT &= ~currentDDR;  // Clear the corresponding bits in PORT
  }
}
/*
// Not needed anymore and does not work reliably with modbus
int getPinMode(int pin) {
  // Determine the port and bit numbers for the specified pin
  volatile uint8_t* ddrRegister = portModeRegister(digitalPinToPort(pin));
  uint8_t bit = digitalPinToBitMask(pin);

  // Check the corresponding bit in the DDRx register
  if ((*ddrRegister & bit) == 0) {
    return INPUT;  // Bit is clear, pin is set as an input
  } else {
    return OUTPUT;  // Bit is set, pin is set as an output
  }
}
*/

/**
* Resets Arduoino
*/
void(* resetFunc) (void) = 0;

void loop() {
  // poll for Modbus RTU requests
  int packetReceived = ModbusRTUServer.poll();
  // Register changed by client
  if(packetReceived) {
    // Checks is Reset holding register 1 if it is let's reset Arduino
    if(ModbusRTUServer.holdingRegisterRead(0x1C) == 1){
      resetFunc();
    }
    if(checkArduinoRegConf()){
      setArduinoRegs();
    }
    // Update each pin same as holding registers (after we have received data that holding registers has been changed by client)
    updateArduinoPins();
  } else {
    // Read digital input pins and set the values to holding registers
    for (int i = DIGI_START_REG; i < DIGI_START_REG + DIGI_COUNT; i++) {
      int pin = i-1;
      ModbusRTUServer.holdingRegisterWrite(i, digitalRead(pin));
    }
    // Read analog input pins and set the values to holding registers
    for (int i = ANALOG_START_REG; i < ANALOG_START_REG + ANALOG_COUNT; i++) {
      int pin = i-1;
      ModbusRTUServer.holdingRegisterWrite(i, analogRead(pin));
    }
  }
}
