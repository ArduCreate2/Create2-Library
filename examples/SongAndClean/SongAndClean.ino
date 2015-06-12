
 /*
   SongAndClean
   Starts the Create 2, plays a short song, and then starts a brief cleaning cycle.
   
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
   
   // The notes and durations of a song
   uint8_t theSong[] = {55,32,30,16,71,8,69,8,71,32,66,32,54,32,30,16,72,8,71,8,72,16,71,16,69,32};
   // Sets song 1 to be the above specified song
   create.song(1,theSong,13);
   // Plays the newly created song
   create.playSong(1);
   
   delay(4500); // Delay for the duration of the song
   
   create.clean(); // Starts normal cleaning cycle
   
   delay(20000); // Clean for 20 seconds
   
   // Stop the Create
   create.stop();
   delay(100);
   create.power();
 }

 void loop() {
   // put your main code here, to run repeatedly
 }
