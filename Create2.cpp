#include <Create2.h>



/*
 * Create2.cpp Library for iRobot Roomba Create2
 * Created by Duncan Lilley and Susan Tuvell
 * Last edit 5/12/2015
 */

Create2::Create2(HardwareSerial* serial, Baud baud){
  _serial = serial;
  _baud = baudCodeToBaudRate(baud);
  _serial->begin(_baud);
  _pollState = PollStateIdle;
}

// Forces the Create 2 to set baud rate to 19200
void Create2::setBaudDefault(uint8_t baudPin) {
  delay(2000);
  pinMode(baudPin,OUTPUT);
  digitalWrite(baudPin,HIGH);
  delay(250);
  for(int i = 1; i <= 3; i++) {
    digitalWrite(baudPin,LOW);
    delay(250);
    digitalWrite(baudPin,HIGH);
    delay(250);
  }
}


// Reset the Create
void Create2::reset(){
  _serial->write(7);
}

// Start OI
// Changes mode to passive
void Create2::start(){
  _serial->begin(_baud);
  _serial->write(128);
}

uint32_t Create2::baudCodeToBaudRate(Baud baud){
  switch (baud){
    case Baud300:
	    return 300;
	case Baud600:
	    return 600;
	case Baud1200:
	    return 1200;
	case Baud2400:
	    return 2400;
	case Baud4800:
	    return 4800;
	case Baud9600:
	    return 9600;
	case Baud14400:
	    return 14400;
	case Baud19200:
	    return 19200;
	case Baud28800:
	    return 28800;
	case Baud38400:
	    return 38400;
	case Baud57600:
	    return 57600;
	case Baud115200:
	    return 115200;
	default:
	    return 57600;
  }
}

void Create2::baud(Baud baud){
  _serial->write(129);
  _serial->write(baud);
  
  _baud = baudCodeToBaudRate(baud);
  _serial->begin(_baud);
}

// Puts the OI into Safe mode, enabling user control of the Roomba.
// Turns off all LEDs.
// OI can be in any mode to accept this command.
void Create2::safeMode(){
  _serial->write(131);
}

// Puts the OI into Full mode, giving you complete control over the Roomba.
// Turns off the cliff, wheel-drop and internal charger safety features. 
void Create2::fullMode(){
  _serial->write(132);
}

// Powers down Create. The OI can be in any state.
void Create2::power(){
  _serial->write(133);
}

// Starts the Spot cleaning mode
void Create2::spot(){
  _serial->write(134);
}

// Starts the default cleaning mode
void Create2::clean(){
  _serial->write(135);
}

// Starts the Max cleaning mode
void Create2::maxClean(){
  _serial->write(136);
}

// Controls Roomba's drive wheels
void Create2::drive(int16_t velocity, int16_t radius){
  _serial->write(137);
  _serial->write((velocity & 0xff00) >> 8);
  _serial->write(velocity & 0xff);
  _serial->write((radius & 0xff00) >> 8);
  _serial->write(radius & 0xff);
}

// Controls the forward and backward motion of 
//Roomba's main brush, side brush, and vacuum independently.
void Create2::motors(int8_t motorBits){
  _serial->write(138);
  _serial->write(motorBits);
}

// Controls LEDs common to all models of Roomba 600
void Create2::LEDs(uint8_t leds, uint8_t powerColor, uint8_t powerIntensity){
  _serial->write(139);
  _serial->write(leds);
  _serial->write(powerColor);
  _serial->write(powerIntensity);
}

// Lets you specify up to four songs to the OI that you can play at a later time.
void Create2::song(uint8_t songNumber, const uint8_t* data, int length){
  _serial->write(140);
  _serial->write(songNumber);
  _serial->write(length);
  _serial->write(data, length << 1);
}

// Selects a song from the songs added to the Roomba using the Song command.
void Create2::playSong(uint8_t songNumber){
  _serial->write(141);
  _serial->write(songNumber);
}

// Requests the OI to send a packet of sensor data bytes.
boolean Create2::sensors(uint8_t packetID, uint8_t* destination, uint8_t length){
  _serial->write(142);
  _serial->write(packetID);
  return getData(destination, length);
}

// Sends the Roomba to the dock.
void Create2::seekDock(){
  _serial->write(143);
}

// Lets you control the speed of the Roomba's main brush, 
//side brush and vacuum indipentently. 
void Create2::pwmMotors(uint8_t mainBrush, uint8_t sideBrush, uint8_t vacuum){
  _serial->write(144);
  _serial->write(mainBrush);
  _serial->write(sideBrush);
  _serial->write(vacuum);
}

// Controls the forward and backward motion of the Roomba's drive wheels independently. 
void Create2::driveDirect(int16_t rightVelocity, int16_t leftVelocity){
  _serial->write(145);
  _serial->write((rightVelocity & 0xff00) >> 8);
  _serial->write(rightVelocity & 0xff);
  _serial->write((leftVelocity & 0xff00) >> 8);
  _serial->write(leftVelocity & 0xff);
}

//Controls the raw forward and backward motion of Roomba's drive wheels indepentently.
void Create2::drivePWM(int16_t rightPWM, int16_t leftPWM){
  _serial->write(146);
  _serial->write((rightPWM & 0xff00) >> 8);
  _serial->write(rightPWM & 0xff);
  _serial->write((leftPWM & 0xff00) >> 8);
  _serial->write(leftPWM & 0xff);
}

// Starts a stream of data packets. The list of packets requested is sent every 15 ms, 
//which is the rate Roomba used to update data. 
void Create2::stream(const uint8_t* packets, uint8_t numPackets){
  _serial->write(148);
  _serial->write(packets, numPackets);
}

// Asks for a list of sensor packets. The result is returned once, as in the Sensors command. 
// The Roomba returns the packets in the order you specify. 
boolean Create2::queryList(const uint8_t* packetIDs, uint8_t numPackets, uint8_t* destination, uint8_t length){
  _serial->write(149);
  _serial->write(packetIDs, numPackets);
  return getData(destination, length);
}

// Lets you pause and restart the steam without clearing the list of requested packets. 
// pause = 0
// resume = 1
void Create2::streamCommand(StreamCommand state){
  _serial->write(150);
  _serial->write(state);
}

// Controls the state of the scheduling LEDs present on the Roomba 560 and 570.
void Create2::scheduleLEDs(uint8_t weekdayBits, uint8_t schedulingBits){
  _serial->write(162);
  _serial->write(weekdayBits);
  _serial->write(schedulingBits);
}

// Controls the four 7 segment displays on the Rommba 560 and 570.
void Create2::digitLEDsRaw(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0){
  _serial->write(163);
  _serial->write(digit3);
  _serial->write(digit2);
  _serial->write(digit1);
  _serial->write(digit0);
}

// Controls the four 7 segment displays on the Roomba 560 and 570 usign ASCII character codes.
void Create2::digitLEDsASCII(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0){
  _serial->write(164);
  _serial->write(digit3);
  _serial->write(digit2);
  _serial->write(digit1);
  _serial->write(digit0);
}

// Lets you push the Roomba's buttons electronically.
void Create2::buttons(uint8_t buttons){
  _serial->write(165);
  _serial->write(buttons);
}

// Sends Roomba a new schedule.
// This takes in a 14 slot array filled with hours and minutes for each day of the week
void Create2::schedule(uint8_t days, uint8_t* schedule){
  _serial->write(167);
  _serial->write(days);
  _serial->write(schedule, 14);
}

// Sets Roomba's clock
void Create2::setDayTime(uint8_t day, uint8_t hour, uint8_t minute){
  _serial->write(168);
  _serial->write(day);
  _serial->write(hour);
  _serial->write(minute);
}

// Returns false if the Roomba has timed out
// Returns true if the Roomba has not timed out
boolean Create2::getData(uint8_t* destination, uint8_t length){
  while(length-- > 0){
    unsigned long startTime = millis();
    while (!_serial->available()){
      //look for timeout
      if (millis() > startTime + CREATE2_READ_TIMEOUT){
	return false; //timed out
      }
    }
    *destination++ = _serial->read();
  }
  return true;
}

// Stops the OI. 
void Create2::stop(){
  _serial->write(173);
}

// State machine to read sensor data and discard everything else
boolean Create2::pollSensors(uint8_t* destination, uint8_t length){
  while(_serial->available()){
    uint8_t ch = _serial->read(); //ch is... ???
    switch (_pollState){
      case PollStateIdle:
	if(ch == 19)
	  _pollState = PollStateWaitCount;
	break;
	
      case PollStateWaitCount:
	_pollCheckSum = _pollSize = ch;
	_pollCount = 0;
	_pollState = PollStateWaitBytes;
	break;
	
      case PollStateWaitBytes:
	_pollCheckSum += ch;
	if (_pollCount < length)
	  destination[_pollCount] = ch;
	if (_pollCount++ >=_pollSize)
	  _pollState = PollStateWaitChecksum;
	break;
	
      case PollStateWaitChecksum:
	_pollCheckSum += ch;
	_pollState = PollStateIdle;
	return (_pollCheckSum == 0);
	break;
    }
  }
  return false;
}
