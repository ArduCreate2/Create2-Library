
 /*
   BasicSetup
   This example program gives the general layout used for testing the Create 2.
   
   by Duncan Lilley and Susan Tuvell
  */

 #include <Create2.h>

 // Creates a Create2 instance to communicate over Serial1 at a baud rate of 19200
 Create2 create(&Serial1, Create2::Baud19200);

 const int BAUD_PIN = 8; // The pin attached to the Baud Rate Change pin on the DIN connector

 void setup() {
   create.setBaudDefault(BAUD_PIN); // Sets baud rate of Create 2 to be 19200
   create.start(); // Starts the Create's OI
   create.fullMode(); // Sets the mode to be Full Mode
   // create.safeMode();  // This line can replace the above line

   delay(100);
   
   /*
     One-time commands for the Create 2 go here, after the 
     Create 2 has been started and is in a mode.
     Commands here will be executed once, at the beginning of the program.
    */
   
   // The following lines stop the Create 2. They should be removed
   // if code to command the Create 2 is going to be put into loop()
   create.stop();
   delay(100);
   create.power();
 }

 void loop() {
   /*
     This is where the main code for controlling the Create 2 should go.
     Commands here will be executed repeatedly after the commands in
     setup() are sent.
    */
 }
