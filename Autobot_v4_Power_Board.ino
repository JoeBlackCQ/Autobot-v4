////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      _         _        _           _           _  _     ____                          ____                      _
//     / \  _   _| |_ ___ | |__   ___ | |_  __   _| || |   |  _ \ _____      _____ _ __  | __ )  ___   __ _ _ __ __| |
//    / _ \| | | | __/ _ \| '_ \ / _ \| __| \ \ / / || |_  | |_) / _ \ \ /\ / / _ \ '__| |  _ \ / _ \ / _` | '__/ _` |
//   / ___ \ |_| | || (_) | |_) | (_) | |_   \ V /|__   _| |  __/ (_) \ V  V /  __/ |    | |_) | (_) | (_| | | | (_| |
//  /_/   \_\__,_|\__\___/|_.__/ \___/ \__|   \_/    |_|   |_|   \___/ \_/\_/ \___|_|    |____/ \___/ \__,_|_|  \__,_| v1.00
//
//  Controls the 15-output Power Board for the Autobot v4.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <Wire.h>

byte gReceivedData[3];  //Array for receiving/sending data over I2C

byte gDataArray[18][4]; //Array for storing the status of all outputs. Elements [0] and [1] aren't used (RX&TX),
                        //nor is [13](LEDIN), [14]-[17] are for A0 to A3. This is so that the index of the array always
                        //corresponds to the actual output.

long gTimeArray[18];    //Contains the list of off-times for all outputs
long gTimeBlink;        //Remembers the last millis() when gBlinkOn changed state

bool gNewData  = false; //Indicates that new data has arrived over I2C and must be dealt with
bool gQBlinkOn = false; //Is one global blink line, that changes state every 250ms
bool gHBlinkOn = false; //Is the other global blink line, that changes state every 500ms
byte gBlinkNum = 0;     //Counts blink cycles, to divide gQBlinks by 2 to get gHBlinks




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __                              _   ___             _
//   \ \    _ _ ___ __ _ _  _ ___ __| |_| __|_ _____ _ _| |_
//    \ \  | '_/ -_) _` | || / -_|_-<  _| _|\ V / -_) ' \  _|
//     \_\ |_| \___\__, |\_,_\___/__/\__|___|\_/\___|_||_\__| 32 bits Edition
//                    |_|
//         A master is calling and requesting something
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void requestEvent() {
    byte DataArray[4];                    // Byte array to split integers into 4 bytes for I2C transmission

    /*Send the value to the slave */
/*  DataArray[0] = (ThePot >> 24) & 0xFF; // Split the integer
    DataArray[1] = (ThePot >> 16) & 0xFF;
    DataArray[2] = (ThePot >>  8) & 0xFF;
    DataArray[3] = ThePot & 0xFF;
    Wire.write(DataArray, 4);             // Send it to the master
*/
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __                    _         ___             _
//   \ \    _ _ ___ __ ___(_)_ _____| __|_ _____ _ _| |_
//    \ \  | '_/ -_) _/ -_) \ V / -_) _|\ V / -_) ' \  _|
//     \_\ |_| \___\__\___|_|\_/\___|___|\_/\___|_||_\__| 32 bits Edition
//
//         A master has sent something.
//         Remember to change "RemotePot" to a global variable you can use in your final program
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void receiveEvent(int bytes) {
    //Serial.println("Receiving...");
    /* Read the 4 bytes that we expect to be there, and load them in the global variable */
    while (Wire.available() > 0) {
      gReceivedData[0] = Wire.read();  // the Command #
      gReceivedData[1] = Wire.read();  // the Output #
      gReceivedData[2] = Wire.read();  // the Value #1
      gReceivedData[3] = Wire.read();  // the Value #2
    }
    //Serial.print("gReceivedData[0]:");Serial.println(gReceivedData[0]);
    //Serial.print("gReceivedData[1]:");Serial.println(gReceivedData[1]);
    //Serial.print("gReceivedData[2]:");Serial.println(gReceivedData[2]);
    //Serial.print("gReceivedData[3]:");Serial.println(gReceivedData[3]);
    //Serial.println("---------------");
    gNewData = true;              // indicate that new data has arrived, so the loop() can do its thing
  }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __        _           _    ___   __  __ _____ _
//   \ \    __| |_  ___ __| |__/ _ \ / _|/ _|_   _(_)_ __  ___ _ _ ___
//    \ \  / _| ' \/ -_) _| / / (_) |  _|  _| | | | | '  \/ -_) '_(_-<
//     \_\ \__|_||_\___\__|_\_\\___/|_| |_|   |_| |_|_|_|_\___|_| /__/
//
//         See if the time of any timed funtion is up and, if so, switch off the corresponding output
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void checkOffTimers() {
    //Check all available timers to see if "offTime" has a value, hence an off-time to be checked
    for (byte i = 0; i <= 17; i++) {
      if (gTimeArray[i] > 0) {
        //This timer has a value, check if its time is up...
        if (millis() > gTimeArray[i]) {
          //The time is up, switch the corresponding output off and reset the off-time value
          digitalWrite(i, LOW);
          gTimeArray[i] = 0;
        }
      }
    }
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __                   _      _       ___ _ _      _   _           _    _
//   \ \    _  _ _ __  __| |__ _| |_ ___| _ ) (_)_ _ | |_(_)_ _  __ _| |  (_)_ _  ___ ___
//    \ \  | || | '_ \/ _` / _` |  _/ -_) _ \ | | ' \| / / | ' \/ _` | |__| | ' \/ -_|_-<
//     \_\  \_,_| .__/\__,_\__,_|\__\___|___/_|_|_||_|_\_\_|_||_\__, |____|_|_||_\___/__/
//              |_|                                             |___/
//         Maintains the two global blinking lines switching on & off every 250 ms (gQBlinkOn)/500 ms (gHBlinkOn)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void updateBlinkingLines() {
    if (millis() - gTimeBlink > 250) {
      gQBlinkOn = !gQBlinkOn;
      gBlinkNum++;
      if (gBlinkNum == 2) {
        gHBlinkOn = !gHBlinkOn; //gHBlinkOn is switched every 2 times gQBlinkOn switches
        gBlinkNum = 0;
      }
      gTimeBlink = millis();
    }
  }



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __             _   ___ _ _      _   _           ___ _        _
//   \ \    ___ ___| |_| _ ) (_)_ _ | |_(_)_ _  __ _/ __| |_ __ _| |_ ___ ___
//    \ \  (_-</ -_)  _| _ \ | | ' \| / / | ' \/ _` \__ \  _/ _` |  _/ -_|_-<
//     \_\ /__/\___|\__|___/_|_|_||_|_\_\_|_||_\__, |___/\__\__,_|\__\___/__/
//                                             |___/
//         Sets the different blinking lines states to on or off
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void setBlinkingStates () {

  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   __                     _     _   _ _  ___       _             _
//   \ \    _ _ ___ ___ ___| |_  /_\ | | |/ _ \ _  _| |_ _ __ _  _| |_ ___
//    \ \  | '_/ -_|_-</ -_)  _|/ _ \| | | (_) | || |  _| '_ \ || |  _(_-<
//     \_\ |_| \___/__/\___|\__/_/ \_\_|_|\___/ \_,_|\__| .__/\_,_|\__/__/
//                                                      |_|
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void resetAllOutputs() {
    digitalWrite( 2, LOW); digitalWrite( 4, LOW); digitalWrite( 7, LOW); digitalWrite( 8, LOW); digitalWrite(12, LOW);
    digitalWrite(A0, LOW); digitalWrite(A1, LOW); digitalWrite(A2, LOW); digitalWrite(A3, LOW);
    digitalWrite( 3, LOW); digitalWrite( 5, LOW); digitalWrite( 6, LOW);
    digitalWrite( 9, LOW); digitalWrite(10, LOW); digitalWrite(11, LOW);
  }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//             _                  ____
//    ___  ___| |_ _   _ _ __    / /\ \
//   / __|/ _ \ __| | | | '_ \  | |  | |
//   \__ \  __/ |_| |_| | |_) | | |  | |
//   |___/\___|\__|\__,_| .__/  | |  | |
//                      |_|      \_\/_/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.println("============================");
  Serial.println("   Autobot v4 Power Board");
  Serial.println("============================");

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Set up all the pins                                                                                                    */
  /*------------------------------------------------------------------------------------------------------------------------*/
  Serial.print("Initializing pins......");
  pinMode( 2, OUTPUT);  // Standard digital pins
  pinMode( 4, OUTPUT);
  pinMode( 7, OUTPUT);
  pinMode( 8, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(A0, OUTPUT);  // Analog inputs used as digital outputs, will be addressed with codes 14-17 (A0 = 14)
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  pinMode(A6,  INPUT);  // As the 328P can't use pins A6 & A7 as outputs, define them as inputs, in case they're needed
  pinMode(A7,  INPUT);

  pinMode( 3, OUTPUT);  // PWM digital pins
  pinMode( 5, OUTPUT);
  pinMode( 6, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.println("done.");

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Shut all the outputs off                                                                                               */
  /*------------------------------------------------------------------------------------------------------------------------*/
  Serial.print("Resetting all outputs..");
  resetAllOutputs();
  Serial.println("done.");

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Set the board up on the I2C bus                                                                                        */
  /*------------------------------------------------------------------------------------------------------------------------*/
  Serial.print("Initializing I2C bus...");
  Wire.begin(1);                // Init this processor as slave #1 on the I2C bus
  Wire.onRequest(requestEvent); // Assign event for requests
  Wire.onReceive(receiveEvent); // Assign event for receptions
  Serial.println("done.");

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Init the arrays for them not having any random values                                                                  */
  /*------------------------------------------------------------------------------------------------------------------------*/
  Serial.print("Initializing arrays....");
  for (byte i = 0; i <= 17; i++) {
    for (byte j = 0; j <= 3; j++) {gDataArray[i][j] = 0;}
    gTimeArray[i] = 0;
  }
  Serial.println("done.");

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Init the blinking variable                                                                                             */
  /*------------------------------------------------------------------------------------------------------------------------*/
  gTimeBlink = millis();

  /*------------------------------------------------------------------------------------------------------------------------*/
  /* Everything is done                                                                                                     */
  /*------------------------------------------------------------------------------------------------------------------------*/
  Serial.println("Ready.");

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    _                      ____
//   | | ___   ___  _ __    / /\ \
//   | |/ _ \ / _ \| '_ \  | |  | |
//   | | (_) | (_) | |_) | | |  | |
//   |_|\___/ \___/| .__/  | |  | |
//                 |_|      \_\/_/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  /************************************************************************************************************************/
  /* If some command has arrived from the master, do what has to be done                                                  */
  /************************************************************************************************************************/
  if (gNewData == true) {
    /* ------------------------------------------------------------------------------------------------------------------ */
    /* FIND OUT WHICH COMMAND HAS TO BE PROCESSED (which is in gReceivedData[0])                                               */
    /* ------------------------------------------------------------------------------------------------------------------ */
    switch (gReceivedData[0]) {
      case 1:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL ON                                                                                            */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "Channel on" gets 2 parameters:                                                                                */
        /*    • gReceivedData[1] is the "output #" which can be: 2-17 (14-17 are analog pins                                   */
        /*    • gReceivedData[2] is the "on time" (in seconds) which is optional. This is the time the output will remain on,  */
        /*      in seconds. Then it will switch off automatically. If "on time" is 0, it's ignored.                       */
        /* -------------------------------------------------------------------------------------------------------------- */

        // Switch the corresponding channel on
        digitalWrite(gReceivedData[1], HIGH);

        //Check if there is timing to be done, if so, calculate and set the time to switch the output off
        if (gReceivedData[2] > 0) {gTimeArray[gReceivedData[1]] = millis() + (gReceivedData[2] * 1000);}
      break;

      case 2:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL OFF                                                                                           */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "Channel off" gets 1 parameter:                                                                                */
        /*    • gReceivedData[1] is the "output #" which can be: 2-17 (14-17 are analog pins                                   */
        /*                                                                                                                */
        /* "Channel off" also switches off any ongoing timing or blinking functions, cutting them short if necessary      */
        /* -------------------------------------------------------------------------------------------------------------- */

        // Switch the corresponding channel off
        digitalWrite(gReceivedData[1], LOW);

        //    _   _   _
        //   | | | | | |
        //   |_| |_| |_|
        //   (_) (_) (_)
        //
        //   ToDo: cancel any waiting or blinking for this output as well

      break;

      case 3: //blink a channel
        gDataArray[gReceivedData[1]][1] = gReceivedData[2];
      break;

      case 4: //set a PWM value for a channel
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL ON WITH A PWM VALUE                                                                           */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "PWM Channel on" gets 3 parameters:                                                                            */
        /*    • gReceivedData[1] is the "output #" which can be: 3,5,6,9,10,11                                                 */
        /*    • gReceivedData[2] is the "pwm value" from 0-255                                                                 */
        /*    • gReceivedData[3] is the "on time" (in seconds) which is optional. This is the time the output will remain on,  */
        /*      in seconds. Then it will switch off automatically. If "on time" is 0, it's ignored.                       */
        /* -------------------------------------------------------------------------------------------------------------- */

        // Filter out all the non PWM outputs
        if (gReceivedData[1] <  3) {break;}
        if (gReceivedData[1] == 4) {break;}
        if (gReceivedData[1] == 7) {break;}
        if (gReceivedData[1] == 8) {break;}
        if (gReceivedData[1] > 11) {break;}

        // Set the output
        //Serial.println("PWM set.");
        analogWrite(gReceivedData[1], gReceivedData[2]);

        // Check if there is timing to be done, if so, calculate and set the time to switch the output off
        if (gReceivedData[3] > 0) {gTimeArray[gReceivedData[1]] = millis() + (gReceivedData[3] * 1000);}
      break;

      case 5: //PWM sweep
        // Filter out all the non PWM outputs
        if (gReceivedData[1] <  3) {break;}
        if (gReceivedData[1] == 4) {break;}
        if (gReceivedData[1] == 7) {break;}
        if (gReceivedData[1] == 8) {break;}
        if (gReceivedData[1] > 11) {break;}

      break;

      case 6: //PWM sweep up
        // Filter out all the non PWM outputs
        if (gReceivedData[1] <  3) {break;}
        if (gReceivedData[1] == 4) {break;}
        if (gReceivedData[1] == 7) {break;}
        if (gReceivedData[1] == 8) {break;}
        if (gReceivedData[1] > 11) {break;}

      break;

      case 7: //PWM sweep down
        // Filter out all the non PWM outputs
        if (gReceivedData[1] <  3) {break;}
        if (gReceivedData[1] == 4) {break;}
        if (gReceivedData[1] == 7) {break;}
        if (gReceivedData[1] == 8) {break;}
        if (gReceivedData[1] > 11) {break;}

      break;

      case 8: //Light sweep upper

      break;

      case 9: //Light sweep lower

      break;

      case 99:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* RESET ALL THE OUTPUTS TO 0                                                                                     */
        /* -------------------------------------------------------------------------------------------------------------- */
        resetAllOutputs();
      break;

      default:
      break;
    }
    gNewData = false;
  }

  /************************************************************************************************************************/
  /* Keep the blinking lines blinking...                                                                                  */
  /************************************************************************************************************************/
  updateBlinkingLines();
  setBlinkingStates();

  /************************************************************************************************************************/
  /* Check if timed stuff has to be done and some output has to be switched off                                           */
  /************************************************************************************************************************/
  checkOffTimers();

}
