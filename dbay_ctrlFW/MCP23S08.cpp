/**********************************************************************
 * 
 * This is the C++ part of the MCP23S08 library.
 * See MCP23S08.h and the example files for a full documentation.
 * 
 *********************************************************************/


//#include "Arduino.h"
#include "MCP23S08.h"



/*##################################### PUBLIC FUNCTIONS #####################################*/

MCP23S08::MCP23S08(uint8_t csPin) : csPin(csPin) {}


MCP23S08::MCP23S08(uint8_t csPin, uint8_t deviceAddr) : csPin(csPin) {
	deviceOpcode |= ((deviceAddr & 0x03) << 1);
}

//MCP23S08::MCP23S08(uint8_t eadr0,uint8_t eadr1,uint8_t eadr2,uint8_t eadr3,uint8_t eadr4,uint8_t eadr5,uint8_t eadr6,uint8_t eadr7) : adr0(eadr0) ,adr1(eadr1),adr2(eadr2),adr3(eadr3),adr4(eadr4),adr5(eadr5),adr6(eadr6),adr7(eadr7) {}

void MCP23S08::begin() {
	SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
	//pinMode(csPin, OUTPUT);
	digitalWrite(csPin, LOW);
	//this->CSbus(false);
	// reset all registers to default:
	SPI.transfer(MCP23S08_IODIR);	//set address pointer to first register
	SPI.transfer(0xFF);				// reset first register
	for (uint8_t i = 0; i < MCP23S08_OLAT; i++) {
		SPI.transfer(0x00);			// reset other 10 registers
	}
	digitalWrite(csPin, HIGH);
	//this->CSbus(true);
	SPI.endTransaction();
}

/*void MCP23S08::CSbus(bool s){
	if(s){
			digitalWrite(adr0, HIGH);
			digitalWrite(adr1, HIGH);
			digitalWrite(adr2, HIGH);
			digitalWrite(adr3, HIGH);
			digitalWrite(adr4, HIGH);
			digitalWrite(adr5, HIGH);
			digitalWrite(adr6, HIGH);
			digitalWrite(adr7, HIGH);
	}else{
			digitalWrite(adr0, LOW);
			digitalWrite(adr1, LOW);
			digitalWrite(adr2, LOW);
			digitalWrite(adr3, LOW);
			digitalWrite(adr4, LOW);
			digitalWrite(adr5, LOW);
			digitalWrite(adr6, LOW);
			digitalWrite(adr7, LOW);
	}
}*/

bool MCP23S08::digitalReadIO(uint8_t pin) {
	if (pin > 7) {
		return 0;
	}
	return (getInputStates() >> pin) & 1;
}


void MCP23S08::digitalWriteIO(uint8_t pin, bool state) {
	if (pin > 7) {
		return;
	}
	
	setOutputStates((getOutputStates() & ~(1 << pin)) | (state << pin));
}


void MCP23S08::pinModeIO(uint8_t pin, uint8_t mode) {
	if (pin > 7) {
		return;
	}
	
	switch (mode) {
		case INPUT:
			setPinModes(getPinModes() & ~(1 << pin));			// set pin to input
			enablePullups(getEnabledPullups() & ~(1 << pin));	// disable pullup for pin
			break;
		case OUTPUT:
			setPinModes(getPinModes() | (1 << pin));				// set pin to output
			enablePullups(getEnabledPullups() & ~(1 << pin));	// disable pullup for pin
			break;
		case INPUT_PULLUP:
			setPinModes(getPinModes() & ~(1 << pin));			// set pin to input
			enablePullups(getEnabledPullups() | (1 << pin));	// enable pullup for pin
			break;
	}
}


void MCP23S08::setOutputStates(uint8_t states) {
	writeRegister(MCP23S08_OLAT, states);
}


void MCP23S08::setPinModes(uint8_t modes) {
	writeRegister(MCP23S08_IODIR, ~(modes));	// inverted to match IDE defaults
}


void MCP23S08::enablePullups(uint8_t enables) {
	writeRegister(MCP23S08_GPPU, enables);
}


uint8_t MCP23S08::getInputStates() {
	return readRegister(MCP23S08_GPIO);
}


uint8_t MCP23S08::getOutputStates() {
	return readRegister(MCP23S08_OLAT);
}


uint8_t MCP23S08::getPinModes() {
	return ~(readRegister(MCP23S08_IODIR));		// inverted to match IDE defaults
}


uint8_t MCP23S08::getEnabledPullups() {
	return readRegister(MCP23S08_GPPU);
}


/*##################################### PRIVATE FUNCTIONS #####################################*/

void MCP23S08::writeRegister(uint8_t address, uint8_t data) {
	SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
	digitalWrite(csPin, LOW);
	//this->CSbus(false);
	SPI.transfer(deviceOpcode);		// initialize transfer with opcode and R/W-flag cleared
	SPI.transfer(address);
	SPI.transfer(data);
	digitalWrite(csPin, HIGH);
	//this->CSbus(true);
	SPI.endTransaction();
}


uint8_t MCP23S08::readRegister(uint8_t address) {
	uint8_t data;
	SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
	digitalWrite(csPin, LOW);
	//this->CSbus(false);
	SPI.transfer(deviceOpcode | 1);		// initialize transfer with opcode and R/W-flag set
	SPI.transfer(address);
	data = SPI.transfer(0);
	digitalWrite(csPin, HIGH);
	//this->CSbus(true);
	SPI.endTransaction();
	return data;
}
