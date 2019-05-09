// Create2.h
//
/// \mainpage Create2 library for Arduino
/// 
/// This is the Arduino Create2 Library
/// It provides an object-oriented interface for communicating with the iRobot Create 2 robot
/// through a serial port.
/// The Roomba is the original device from iRobot.
/// The Create 2 is a new version of the iRobot Create, which is a platform for hobbyists. It has a
/// serial port DIN socket underneath the plastic cover on the top of the robot. 
/// 
/// The Create 2 uses a set of commands supported by this library. 
/// 
/// The version of the package that this documentation refers to can be downloaded 
/// <a href="Create2.zip"><b>here</b></a>.
/// 
/// This library has been tested using an Arduino Mega 2560.
/// 
/// The following example programs have been included to demonstrate how functions from the Create2 Library can be used.
/// 
/// \li <B>BasicSetup:</B> Demonstrates the general layout of code we used to control the Create 2
/// \li <B>Sensors:</B> Shows how sensor packets are retrieved from the Create 2
/// \li <B>SongAndClean:</B> Demo of how to to achieve output from the Create 2
/// \li <B>Recharge:</B> Gives an example of using sensor packets to make decisions
///
/// \par Open Interface
/// The iRobot Open Interface (OI) for the Create 2 defines communication with the Create 2.
/// A copy of the OI can be found <a href="Create_2_OI.pdf"><b>here</b></a>.
/// 
/// \par Installation
/// To install this library, unzip the downloaded zip file and place the directory it contains into
/// the libraries folder of your sketchbook.
/// More detailed instructions to install Arduino Libraries can be found at http://www.arduino.cc/en/guide/libraries
/// 
/// This software is Copyright (C) 2015 Susan Tuvell and Duncan Lilley.
/// Adaptation from Roomba.h Arduino Library Copyright (C) 2010 Mike McCauley (mikem@airspayce.com)
/// Use is subject to license conditions. The main licensing options available are GPL V2 or Commercial:
///
/// \par Open Source Licensing GPL V2
/// This is the appropriate option if you want to share the source code of your
/// application with everyone you distribute it to, and you also want to give them
/// the right to share who uses it. If you wish to use this software under Open
/// Source Licensing, you must contribute all your source code to the open source
/// community in accordance with the GPL Version 2 when your application is
/// distributed. See http://www.gnu.org/copyleft/gpl.html
/// 
/// \par Commercial Licensing
/// This is the appropriate option if you are creating proprietary applications
/// and you are not prepared to distribute and share the source code of your
/// application. Contact info@airspayce.com for details.
///
/// \par Revision History
/// \version 1.0 Initial Release
/// 
/// \author Susan Tuvell (susan.tuvell@hws.edu) 
/// \author Duncan Lilley (duncan.lilley@hws.edu)
/// \author Thanks to Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2015 Susan Tuvell and Duncan Lilley

#ifndef Create2_h
#define Create2_h

#if (ARDUINO < 100)
#include "WProgram.h
#else
#include <Arduino.h>
#endif

/// Masks for LEDs in leds()
#define CREATE2_MASK_LED_NONE 		0x0
#define CREATE2_MASK_LED_DEBRIS 	    	0x1
#define CREATE2_MASK_LED_SPOT 		0x2
#define CREATE2_MASK_LED_DOCK 		0x4
#define CREATE2_MASK_LED_CHECKROBOT 	0x8

/// Mask for Roomba only motors:
#define CREATE2_MASK_MOTOR_SIDE 	 	0x1
#define CREATE2_MASK_MOTOR_VACUUM 	0x2
#define CREATE2_MASK_MOTOR_MAIN 		0x4
#define CREATE2_MASK_MOTOR_SIDE_DIR 	0x8
#define CREATE2_MASK_MOTOR_MAIN_DIR 	0x10

/// Masks for bumps and wheedrops sensor packet ID 7
#define CREATE2_MASK_BUMP_RIGHT 		0x1
#define CREATE2_MASK_BUMP_LEFT 		0x2
#define CREATE2_MASK_WHEELDROP_RIGHT 	0x4
#define CREATE2_MASK_WHEELDROP_LEFT 	0x8

/// Masks for driver overcurrents Packet ID 14
#define CREATE2_MASK_SIDE_BRUSH 		0x1
#define CREATE2_MASK_MAIN_BRUSH 		0x4
#define CREATE2_MASK_RIGHT_WHEEL 		0x8
#define CREATE2_MASK_LEFT_WHEEL 		0x10
// Roomba, use CREATE2_MASK_SIDE_BRUSH,  CREATE2_MASK_VACUUM, CREATE2_MASK_MAIN_BRUSH

/// Masks for buttons sensor packet ID 18
#define CREATE2_MASK_BUTTON_CLEAN 	0x1
#define CREATE2_MASK_BUTTON_SPOT 		0x2
#define CREATE2_MASK_BUTTON_DOCK 		0x4
#define CREATE2_MASK_BUTTON_MIN 		0x8
#define CREATE2_MASK_BUTTON_HOUR 		0x10
#define CREATE2_MASK_BUTTON_DAY 		0x20
#define CREATE2_MASK_BUTTON_SCHEDULE 	0x40
#define CREATE2_MASK_BUTTON_CLOCK 	0x80 

// Masks for charging sources sensor packet ID 34
#define CREATE2_MASK_INTERNAL_CHARGER 	0x1
#define CREATE2_MASK_HOME_BASE         0x2

/// \def CREATE2_READ_TIMEOUT
/// Read timeout in milliseconds.
/// If we have to wait more than this to read a char when we are expecting one, then something is wrong.
#define CREATE2_READ_TIMEOUT 		200

/////////////////////////////////////////////////////////////////////
/// \class Create2 Create2.h <Create2.h>
/// \brief Support for the iRobot Create 2 via serial port using the iRobot Open Interface (OI) protocol.
/// 
/// The iRobot Create 2 platform supports a serial port through which one can communicate with the robot.
/// The protocol implemented here conforms to the Open Interface protocol described in the iRobot Open
/// Interface Command Reference. This can be found <a href="Create_2_OI.pdf"><b>here</b></a>.
/// 
/// The Create 2 is equipped with a mini-DIN serial port socket.
/// 
/// In order to communicate with a Create 2, you must create an instance of the Create2 class and then call its
/// instance methods to send commands and receive sensor messages. You can also request sensor data to be 
/// retrieved from the Create 2.
/// 
/// \par Electrical Considerations
///
/// The serial port output TXD from the Roomba/Create is too weak to drive the RX serial port input 
/// of an Arduino properly.
/// This is because of the USB-Serial converter on the Arduino: it also tries to drive the RX serial port input
/// via a pullup resistor, 
/// but the Roomba does not have enough drive to pull the RX down below about 2.5 volts, which is insufficient 
/// to be reliably detected as a TTL serial input of 0.
///
/// \note Note that this problem does not affect the Serial1 Serial2 or Serial3 ports on the Mega: these ports do not 
/// have other circuitry connected to them and the Roomba can drive the serial port inputs of these ports 
/// without a problem. Its only the RX Serial port (ie the first Serial port) that is affected by this problem. 
///
/// What this means is that if you intend to connect a Roomba to the standard (first) RX/TX serial port on an Arduino 
/// you \a MUST have additional circuitry to help drive RX on the Arduino low enough. 
/// 

class Create2{
  public:

    /// \enum Baud
    /// Demo types to pass to Create2::baud()
    typedef enum{
      Baud300 				= 0,
      Baud600 				= 1,
      Baud1200 				= 2,
      Baud2400 				= 3,
      Baud4800 				= 4,
      Baud9600 				= 5,
      Baud14400 				= 6,
      Baud19200 				= 7,
      Baud28800 				= 8,
      Baud38400 				= 9,
      Baud57600 				= 10,
      Baud115200 			= 11,
    } Baud;

    /// \enum Drive
    /// Special values for radius in Create2::drive()
     typedef enum{
       DriveStraight 				= 0x8000,
       DriveInPlaceClockwise 		= 0xFFFF,
       DriveInPlaceCounterClockwise 	= 0x0001,
    } Drive;

    /// \enum StreamCommand 
    /// Values to pass to Create2::streamCommand()
    typedef enum{
      StreamCommandPause 			= 0,
      StreamCommandResume 			= 1,
    } StreamCommand;

    /// \enum IRCommand
    /// Values for sensor packet ID 27
    typedef enum{
      //IR Remote Control
      IRCommandLeft 				= 129,
      IRCommandForward 			= 130,
      IRCommandRight 				= 131,
      IRCommandSpot 				= 132,
      IRCommandMax 				= 133,
      IRCommandSmall 				= 134,
      IRCommandMedium 				= 135,
      IRCommandLargeClean 			= 136,
      IRCommandStop 				= 137,
      IRCommandPower 				= 138,
      IRCommandArcForwardLeft 		= 139,
      IRCommandArcForwardRight 		= 140,
      IRCommandDriveStop 			= 141,
      // Scheduling Remote:
      IRCommandDownload 			= 142,
      IRCommandSeekDock 			= 143,
      // Roomba Discovery Drive on Charge:
      IRCommandReserved1 			= 240,
      IRCommandRedBuoy 			= 248,
      IRCommandGreenBuoy 			= 244, 
      IRCommandForceField 			= 242,
      IRCommandRedGreenBuoy 		= 252,
      IRCommandRedBuoyForceField 		= 250,
      IRCommandGreenBuoyForceField 	= 246,
      IRCommandRedGreenBuoyForceField 	= 254,
    } IRCommand;

    /// \enum ChargeState
    /// Values for sensor packet ID 21
    typedef enum{
      ChargeStateNotCharging 			= 0,
      ChargeStateReconditioningCharging 	= 1,
      ChargeStateFullChanrging 			= 2,
      ChargeStateTrickleCharging 			= 3,
      ChargeStateWaiting 				= 4,
      ChargeStateFault 				= 5,
    } ChargeState;

    /// \enum OIMode
    /// Values for sensor packet ID 35
    typedef enum{
      ModeOff     			= 0,
      ModePassive 			= 1,
      ModeSafe    			= 2,	
      ModeFull    			= 3,
    } OIMode;
    
    /// \enum Sensor
    /// Values for sensor packet IDs to pass to sensors() and queryList()
    typedef enum{
      Sensors7to26 				= 0,
      Sensors7to16 				= 1,
      Sensors17to20 				= 2,
      Sensors21to26 				= 3,
      Sensors27to34 				= 4,
      Sensors35to42 				= 5,
      Sensors7to42 				= 6,
      Sensors7to58 				= 100,
      Sensors43to58 				= 101,
      Sensors46to51 				= 106,
      Sensors54to58 				= 107,
      SensorBumpsAndWheelDrops 		= 7,
      SensorWall 				= 8,
      SensorCliffLeft 				= 9,
      SensorCliffFrontLeft 			= 10,
      SensorCliffFrontRight 		= 11,
      SensorCliffRight 			= 12,
      SensorVirtualWall 			= 13,
      SensorOvercurrents 			= 14,
      SensorDirtDetect 			= 15,
//    SensorUnused1 				= 16,
      SensorIRopCode 				= 17,
      SensorButtons 				= 18,
      SensorDistance 				= 19,
      SensorAngle 				= 20,
      SensorChargingState 			= 21,
      SensorVoltage 				= 22,
      SensorCurrent 				= 23,
      SensorTemperature 			= 24,
      SensorBatteryCharge 			= 25,
      SensorBatteryCapacity 		= 26,
      SensorWallSignal 			= 27,
      SensoCliffLeftSignal 			= 28,
      SensoCliffFrontLeftSignal 		= 29,
      SensoCliffFrontRightSignal 		= 30,
      SensoCliffRightSignal 		= 31,
//    SensorUnused2	 			= 32,
//    SensorUnused3 				= 33,
      SensorChargerAvailable 		= 34,
      SensorOIMode 				= 35,
      SensorSongNumber 			= 36,
      SensorSongPlaying 			= 37,
      SensorOIStreamNumPackets 		= 38,
      SensorVelocity 				= 39,
      SensorRadius 				= 40,
      SensorRightVelocity 			= 41,
      SensorLeftVelocity 			= 42,
      SensorEncoderCountsLeft 		= 43,
      SensorEncoderCountsRight 		= 44,
      SensorLightBumper 			= 45,
      SensorLightBumperLeft 		= 46,
      SensorLightBumperFrontLeft 		= 47,
      SensorLightBumperCenterLeft 	= 48,
      SensorLightBumperCenterRight 	= 49,
      SensorLightBumperFrontRight 	= 50,
      SensorLightBumperRight 		= 51,
      SensorIRopCodeLeft 			= 52,
      SensorIRopCodeRight 			= 53,
      SensorLeftMotorCurrent 		= 54,
      SensorRightMotorCurrent 		= 55,
      SensorMainBrushCurrent 		= 56,
      SensorSideBrushCurrent 		= 57,
      SensorStasis	 			= 58,
    } Sensor;
    
    
    /// Constructor. Initializes a new Create2 object to represent a Create 2
    /// \param[in] serial Pointer to the HardwareSerial port to use to communicate with the Roomba. 
    /// Defaults to &Serial
    /// \param[in] baud the baud rate to use on the serial port. 
    /// Defaults to 57600, the default for the Roomba.
    Create2(HardwareSerial* serial = &Serial, Baud baud = Baud57600);

    // Create 2 commands
    
    /// Resets the Create 2. 
    /// It will emit its startup message
    /// Caution, this may take several seconds to complete
    void reset(); //opCode-7
    
    /// Starts the Open Interface and sets the mode to Passive. 
    /// You must send this before sending any other commands.
    /// Initialises the serial port to the baud rate given in the constructor
    void start(); //opCode-128
    
    /// Converts the specified baud code into a baud rate in bits per second
    /// \param[in] baud Baud code, one of Create2::Baud
    /// \return baud rate in bits per second
    // Helper method
    uint32_t baudCodeToBaudRate(Baud baud);
    
    /// Changes the baud rate
    /// \param[in] baud one of the Create2::Baud
    /// The desired baud rate
    void baud(Baud baud); //opCode-129. 
  
    /// Sets the OI to Safe mode.
    /// In Safe mode, the cliff and wheel drop detectors work to prevent Roomba driving off a cliff
    void safeMode(); //opCode-131
    
    /// Sets the OI to Full mode.
    /// In Full mode, the cliff and wheel drop detectors do not stop the motors: you are responsible
    /// for full control of the Roomba
    void fullMode(); //opCode-132
    
    /// Puts a Roomba in sleep mode.
    ///
    void power(); //opCode-133
    
    /// Starts the spot cover mode
    /// Changes mode to Passive
    void spot(); //opCode-134
    
    /// Starts the default cleaning mode
    /// Changes mode to Passive
    void clean(); //opCode-135
    
    /// Starts the max cleaning mode
    /// Changes mode to Passive
    void maxClean(); //opCode-136
    
    /// Starts the Create 2 driving with a specified wheel velocity and radius of turn
    /// \param[in] velocity Speed in mm/s 
    /// -500 to 500
    /// \param[in] radius Radius of the turn in mm. 
    /// -2000 to 2000
    /// Any of the special values in enum Create2::Drive may be used instead of a radius value
    void drive(int16_t velocity, int16_t radius); //opCode-137. 
    
    /// Controls the forward and backward motion of Roomba's main brush, side brush, and
    /// vacuum independently. All motors will run at maximum speed when enabled
    /// \param[in] motorBits Bitmask of outputs to enable.
    /// ORed combination of CREATE2_MASK_DRIVER_*
    void motors(int8_t motorBits); //opCode-138. 
    
    /// Controls the LEDs on the Create
    /// \param[in] leds Bitmask specifying which LEDs to activate. 
    /// ORed combination of CREATE2_MASK_LED_*
    /// \param[in] powerColor The colour of the Power LED. 
    /// 0 to 255: 0 = green, 255 = red, intermediate values are intermediate colours
    /// \param[in] powerIntensity Power LED intensity. 
    /// 0 to 255. 0 = off, 255 = full intensity
    void LEDs(uint8_t leds, uint8_t powerColor, uint8_t powerIntensity); //opCode-139. 
    
    /// Defines a song which can later be played with playSong()
    /// \param[in] songNumber Song number for this song. 
    /// 0 to 4
    /// \param[in] data Array of note/duration pairs. See Open Interface manual for details. 
    /// 2 bytes per note, first byte is the note and the second is the duration.
    /// \param[in] length Length of notes array in bytes, so this will be twice the number of notes in the song
    void song(uint8_t songNumber, const uint8_t* data, int length); //opCode-140. 
    
    /// Plays a song that has previously been defined by song()
    /// \param[in] songNumber The song number to play. 
    /// 0 to 15
    void playSong(uint8_t songNumber); //opCode-141. 
    
    /// Reads the sensor data for the specified sensor packet ID. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure len agrees with the expected 
    /// length of the sensor data. See the Open Interface manual for details on sensor packet lengths.
    /// Create2.h defines various enums and defines for decoding sensor data.
    /// Blocks until all len bytes are read or a read timeout occurs.
    /// \param[in] packetID The ID of the sensor packet to read from Create2::Sensor
    /// \param[out] destination Destination where the read data is stored. 
    /// Must have at least length bytes available.
    /// \param[in] length Number of sensor data bytes to read
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    boolean sensors(uint8_t packetID, uint8_t* destination, uint8_t length); //opCode-142.
    
    /// Causes roomba to immediately seek the docking station.
    ///
    void seekDock(); //opCode-143
    
    /// Sets the duty cycle of the PWM outputs for low side driver.
    /// This is used to drive the main brush, side brush, and vacuum at various speeds.
    /// For the main brush and side brush, positive values turn the motor in the default
    /// direction and negative values turn it in the opposite direction. The vacuum only
    /// runs in one direction.
    /// \param[in] mainBrush Duty cycle for the main brush. 
    /// -127 to 127
    /// \param[in] sideBrush Duty cycle for the side brush.
    /// -127 to 127
    /// \param[in] vacuum Duty cycle for the vacuum.
    /// 0 to 127
    void pwmMotors(uint8_t mainBrush, uint8_t sideBrush, uint8_t vacuum); //opCode-144
    
    /// Starts the Roomba driving with a specified velocity for each wheel
    /// \param[in] rightVelocity Right wheel velocity in mm/s 
    /// -500 to 500
    /// \param[in] leftVelocity Left wheel velocity in mm/s 
    /// -500 to 500
    void driveDirect(int16_t rightVelocity, int16_t leftVelocity); //opCode-145
    
    /// Starts the Roomba driving with a specified PWM for each wheel
    /// \param[in] rightPWM Right wheel PWM 
    /// -255 to 255
    /// \param[in] leftPWM Left wheel PWM 
    /// -255 to 255
    void drivePWM(int16_t rightPWM, int16_t leftPWM); //opCode-146
    
    /// Requests that a stream of sensor data packets be sent by the Roomba.
    /// See the Open Interface manual for details on the resutting data.
    /// The packets will be sent every 15ms.
    /// You can use pollSensors() to receive sensor data streams.
    /// Create only. No equivalent on Roomba.
    /// See the Open Interface maual for more details and limitations.
    /// \param[in] packets Array specifying sensor packet IDs from Create2::Sensor to be sent.
    /// \param[in] numPackets Number of IDs in packetIDs
    void stream(const uint8_t* packets, uint8_t numPackets); //opCode-148.
    
    /// Reads the sensor data for the specified set of sensor packet IDs. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure length agrees with the expected 
    /// length of the sensor data. See the Open Interface manual for details on sensor packet lengths.
    /// Blocks until all length bytes are read or a read timeout occurs.
    /// Create only. No equivalent on Roomba.
    /// \param[in] packetIDs Array of IDs from Create2::Sensor of the sensor packets to read
    /// \param[in] numPackets number of IDs in the packetIDs array
    /// \param[out] destination Destination where the read data is stored. 
    /// Must have at least len bytes available.
    /// \param[in] length Number of sensor data bytes to read and store to dest.
    /// \return true if all length bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    boolean queryList(const uint8_t* packetIDs, uint8_t numPackets, uint8_t* destination, uint8_t length); //opCode-149 
    
    /// Pause or resume a stream of sensor data packets previously requested by stream()
    /// \param[in] state One of Create2::StreamCommand. The status of the stream command. 
    /// Either pause (0) or resume (1).
    void streamCommand(StreamCommand state); //opCode-150
    
    /// Controls the state of the scheduleing LEDs on the Roomba.
    /// See the Open Interface for details on the parameters.
    /// No change to mode
    /// \param[in] weekdayBits bitmask for the weekday LEDs
    /// \param[in] schedulingBits bitmask for the scheduling LEDs
    void scheduleLEDs(uint8_t weekdayBits, uint8_t schedulingBits); //opCode-162
    
    /// Controls the four 7-segment displays on the Roomba.
    /// Digits are ordered from left to right on the robot 3,2,1,0.
    /// No change to mode
    /// <B>Note:</B> the OI states that this command should use bitmasks to control the individual
    /// segments of each 7-segment display. However, it instead displays the number which is
    /// passed in as a parameter. So, 0x1 = 1, 0x2 = 2, etc.
    /// \param[in] digit3 Bitmask for the leftmost digit.
    /// \param[in] digit2 Bitmask for the second digit from the left.
    /// \param[in] digit1 Bitmask for the second digit from the right.
    /// \param[in] digit0 Bitmask for the rightmost digit.
    void digitLEDsRaw(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0); //opCode-163
    
    /// Controls the four 7-segment displays on the Roomba using ASCII code.
    /// See the Open Interface for a chart of ASCII values.
    /// No change to mode
    /// \param[in] digit3 ASCII code for the leftmost display.
    /// \param[in] digit2 ASCII code for the second display from the left.
    /// \param[in] digit1 ASCII code for the second display from the right.
    /// \param[in] digit0 ASCII code for the rightmost display.
    void digitLEDsASCII(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0); //opCode-164
    
    /// Lets you push Roomba's buttons. Code equilalent of pushing a button. 
    /// Button will automatically release after 1/6th of a second
    /// No change to mode
    /// \param[in] buttons Bitmask for the button being 'pressed'.
    /// ORed combination of CREATE2_MASK_BUTTON_*
    void buttons(uint8_t buttons); //opCode-165
    
    /// This command sends Roomba a new schedule. 
    /// To clear the schedule, send all 0s.
    /// No change to mode
    /// \param[in] days Bitmask for which days the schedule is being set
    /// \param[in] schedule Array holding the hour and minute for the schedule for each day.
    /// This is in the format [Sun Hour][Sun Minute][Mon Hour][Mon Minute]...[Sat Hour][Sat Minute]
    void schedule(uint8_t days, uint8_t* schedule); //opCode-167
    
    /// This command sets Roomba's clock
    /// No change to mode
    /// \param[in] day The day of the week
    /// 0 (Sunday) to 6 (Saturday)
    /// \param[in] hour The current hour
    /// 0 to 23
    /// \param[in] minute The current minute
    /// 0 to 59
    void setDayTime(uint8_t day, uint8_t hour, uint8_t minute); //opCode-168
    
    /// Low level function to read length bytes of data from the Roomba
    /// Blocks until all length bytes are read or a read timeout occurs.
    /// \param[out] destination Destination where the read data is stored. 
    /// Must have at least length bytes available.
    /// \param[in] length Number of bytes to read
    /// \return true if all length bytes were successfully read. Returns false 
    /// in the case of a timeout on reading any byte.
    boolean getData(uint8_t* destination, uint8_t length);
    
    /// Stops the OI - stops all streams and the robot will no longer take commands.
    /// Changes mode to off
    void stop(); //opCode-173

    /// Polls the serial input for data belonging to a sensor data stream previously requested with stream().
    /// As sensor data is read it is appended to dest until at most length bytes are stored there. 
    /// When a complete sensor stream has been read with a correct checksum, returns true. 
    /// See the Open Interface manual for details on how the sensor data will be encoded in destination.
    /// Discards characters that are not part of a stream, such as the messages the Roomba 
    /// sends at startup and while charging.
    /// Create only. No equivalent on Roomba.
    /// \param[out] destination Destination where the read data is stored. 
    /// Must have at least length bytes available.
    /// \param[in] length Max number of sensor data bytes to store to destination
    /// \return true when a complete stream has been read, and the checksum is correct. The sensor data
    /// (at most length bytes) will have been stored into destination, ready for the caller to decode.
    boolean pollSensors(uint8_t* destination, uint8_t length);

    /// Forces the Create 2 to change its baud rate to the default rate 19200.
    /// \param[in] baudPin The pin attached to the Baud Rate Change DIN pin.
    void setBaudDefault(uint8_t baudPin);

private:
    /// \enum PollState
    /// Values for _pollState
    typedef enum{
      PollStateIdle		= 0,
      PollStateWaitCount	= 1,
      PollStateWaitBytes 	= 2,
      PollStateWaitChecksum	= 3,
    } PollState;
    
    //helper methods
    uint32_t _baud; /// The baud rate to use for the serial port
    HardwareSerial* _serial; /// The serial port to use to talk to the Roomba
  
    /// Variables for keeping track of polling of data streams
    
    uint8_t 	_pollState; /// Current state of polling
    
    uint8_t 	_pollSize; /// Expected size of the data stream in bytes
    
    uint8_t 	_pollCount; /// Num of bytes read so far
    
    uint8_t 	_pollCheckSum; /// Running checksum counter of data bytes + counts 
};

#endif
