
 /*
   Recharge
   Starts the Create 2, runs a short cleaning cycle, then seeks out the charging dock.
   
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
   delay(100);
   
   create.clean(); // Starts normal cleaning cycle

   delay(5000); // Clean for 5 seconds
   
   create.fullMode();
   delay(100);
   
   create.seekDock(); // Signals the Create to find the charging dock
   boolean seekingDock = true;
   
   while(seekingDock) {
     uint8_t chargeState;
     create.sensors(Create2::SensorChargerAvailable,&chargeState,1);
     
     if(chargeState > 0) {
       // Charging source found, stop the Create
          seekingDock = false;
          create.stop();
          delay(100);
          create.power();
     }
     delay(100);
   }
 }

 void loop() {
   // put your main code here, to run repeatedly
 }
