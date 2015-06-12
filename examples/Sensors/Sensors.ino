
 /*
   Sensors
   Starts the Create 2, demonstrates how sensor packets are retrieved, and then
   prints the values retrieved out through the default Serial port.
   
   by Duncan Lilley and Susan Tuvell
  */

 #include <Create2.h>

 // Creates a Create2 instance to communicate over Serial1 at a baud rate of 19200
 Create2 create(&Serial1, Create2::Baud19200);

 uint8_t sensors[80];
 const int BAUD_PIN = 8; // The pin attached to the Baud Rate Change pin on the DIN connector

 void setup() {
   create.setBaudDefault(BAUD_PIN); // Sets baud rate of Create 2 to be 19200
   
   create.start(); // Starts the Create's OI
   create.fullMode(); // Sets the mode to be Full Mode
   
   create.sensors(Create2::Sensors7to58,sensors,80);
   
   Serial.begin(9600);
   for(uint8_t i = 0; i < 80; i++) {
     Serial.print("Sensor value in index "); Serial.print(i); Serial.print(":\t");
     Serial.println(sensors[i]);
   }
 }

 void loop() {
   // put your main code here, to run repeatedly
 }
