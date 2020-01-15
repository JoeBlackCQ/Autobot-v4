////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      _         _        _           _           _  _     ____                          ____                      _
//     / \  _   _| |_ ___ | |__   ___ | |_  __   _| || |   |  _ \ _____      _____ _ __  | __ )  ___   __ _ _ __ __| |
//    / _ \| | | | __/ _ \| '_ \ / _ \| __| \ \ / / || |_  | |_) / _ \ \ /\ / / _ \ '__| |  _ \ / _ \ / _` | '__/ _` |
//   / ___ \ |_| | || (_) | |_) | (_) | |_   \ V /|__   _| |  __/ (_) \ V  V /  __/ |    | |_) | (_) | (_| | | | (_| |
//  /_/   \_\__,_|\__\___/|_.__/ \___/ \__|   \_/    |_|   |_|   \___/ \_/\_/ \___|_|    |____/ \___/ \__,_|_|  \__,_| v1.10
//
//  Controls the 15-output Power Board for the Autobot v4.
//
//  v1.00 (15.01.2020) First release. Allows switching outputs on, off, pwm, and blinking, all functions support 
//                     auto-shutoff timers. Supports 4 byte commands over I2C.
//	v1.10 (15.01.2020) PWM Sweep (up+down, up, down) functionality implemented.
//
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
          // Cancel any waiting or blinking for this output as well, setting the COMMAND field to 0 for that output
          gDataArray[i][0] = 0;
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
//   __        _           _   ___ _ _      _      _           _ ___
//   \ \    __| |_  ___ __| |_| _ ) (_)_ _ | |__  /_\  _ _  __| / __|_ __ _____ ___ _ __
//    \ \  / _| ' \/ -_) _| / / _ \ | | ' \| / / / _ \| ' \/ _` \__ \ V  V / -_) -_) '_ \
//     \_\ \__|_||_\___\__|_\_\___/_|_|_||_|_\_\/_/ \_\_||_\__,_|___/\_/\_/\___\___| .__/
//                                                                                 |_|
//         Checks if blinking or sweeping is to be done and advances it a step, if so.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void checkBlinkAndSweep () {
		for (byte i = 0; i <= 17; i++) {
			if (gDataArray[i][0] != 0) {
        // SIMPLE BLINKING    //////////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 3) {	// Make this output blink
					if (gDataArray[i][1] == 1) {digitalWrite(i,gQBlinkOn);} else {digitalWrite(i,gHBlinkOn);}	// Fast or slow blink
				}

        // PWM SWEEP UP & DOWN    //////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 5) {	// Make this output sweep up & down with PWM
					/* code */
				}

        // PWM SWEEP UP     ////////////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 6) {	// Make this output sweep up with PWM
					/* code */
				}

        // PWM SWEEP DOWN     //////////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 7) {	// Make this output sweep down with PWM
					/* code */
				}

        // LIGHT SWEEP UPPER     ///////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 8) {	// Make the upper outputs (2, 4, 7, 8, 12) light-sweep back and forth
					/* code */
				}

        // LIGHT SWEEP LOWER     ///////////////////////////////////////////////////////////////////////////////////////////
				if (gDataArray[i][0] == 9) {	// Make the lower outputs (A0, A1, A2, A3) light-sweep back and forth
					/* code */
				}
//				gDataArray[rcvOutput][0] = rcvCommand;	// Store the COMMAND (3 = blink)
//				gDataArray[rcvOutput][1] = rcvValue1;		// Store the SPEED (1 or 2, 250ms or 500ms)
//				gDataArray[rcvOutput][2] = rcvValue2;		// Store the ON TIME (optional, in seconds)
			}
		}

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
	// Make things easier to read assigning friendlier variable names
	byte rcvCommand;
	byte rcvOutput;
	byte rcvValue1;
	byte rcvValue2;

	rcvCommand = gReceivedData[0];	// The command to be executed
	rcvOutput  = gReceivedData[1];	// The output that command is intended for
	rcvValue1  = gReceivedData[2];	// The first parameter, can be SPEED, TIME or PWM VALUE
	rcvValue2  = gReceivedData[3];	// The second parameter, can be TIME or # OF CYCLES


  /************************************************************************************************************************/
  /* If some command has arrived from the master, do what has to be done                                                  */
  /************************************************************************************************************************/
  if (gNewData == true) {
    /* ------------------------------------------------------------------------------------------------------------------ */
    /* FIND OUT WHICH COMMAND HAS TO BE PROCESSED (which is in gReceivedData[0])                                          */
    /* ------------------------------------------------------------------------------------------------------------------ */
    switch (rcvCommand) {
      case 1:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL ON                                                                                            */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "Channel on" uses 1 parameter:                                                                                 */
        /*    • rcvValue2 is the "on time" (in seconds) which is optional. This is the time the output will remain 				*/
				/*      on, in seconds. Then it will switch off automatically. If "on time" is 0, it's ignored.                   */
        /* -------------------------------------------------------------------------------------------------------------- */

        // Switch the corresponding channel on
        digitalWrite(rcvOutput, HIGH);

        //Check if there is timing to be done, if so, calculate and set the time in gTimeArray to switch the output off
        if (rcvValue2 > 0) {gTimeArray[rcvOutput] = millis() + (rcvValue2 * 1000);}
      break;

      case 2:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL OFF                                                                                           */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "Channel off" uses no parameters and also switches off any ongoing timing or blinking functions, cutting				*/
				/* them short if necessary.     																																									*/
        /* -------------------------------------------------------------------------------------------------------------- */

        // Switch the corresponding channel off
        digitalWrite(rcvOutput, LOW);

        // Cancel any waiting or blinking for this output as well, setting the COMMAND field to 0 for that output
				gDataArray[rcvOutput][0] = 0;
      break;

      case 3:
				/* -------------------------------------------------------------------------------------------------------------- */
				/* MAKE A CHANNEL BLINK                                                                                           */
				/* -------------------------------------------------------------------------------------------------------------- */
				/* "Channel blink" uses 2 parameters:                                                                             */
				/*    • rcvValue1 is the SPEED at which the output has to blink and can be "1" (250ms) or "2" (500ms)							*/
				/*    • rcvValue2 is the ON TIME (in seconds) and is optional. This is the time the output will remain on, 				*/
        /*      					in seconds. Then it will switch off automatically. If ON TIME is 0, it's ignored.        				*/
				/*                                                                                                                */
				/* In order to allow the loop() to maintain the blinking functions, gDataArray[] is loaded with the necessary			*/
				/* data, so the blinking can be maintained over time, and/or switched off if necessary.														*/
				/*                                                                                                                */
				/* -------------------------------------------------------------------------------------------------------------- */

				// Load the appropriate gDataArray row (the OUTPUT # in rcvOutput), which will be needed in the loop()
				// to do the actual blinking.
				gDataArray[rcvOutput][0] = rcvCommand;	// Store the COMMAND (3 = blink)
        gDataArray[rcvOutput][1] = rcvValue1;		// Store the SPEED (1 or 2, 250ms or 500ms)
				gDataArray[rcvOutput][2] = rcvValue2;		// Store the ON TIME (optional, in seconds)

        //Check if there is timing to be done, if so, calculate and set the time in gTimeArray to switch the output off
        if (rcvValue2 > 0) {gTimeArray[rcvOutput] = millis() + (rcvValue2 * 1000);}
      break;

      case 4:
        /* -------------------------------------------------------------------------------------------------------------- */
        /* SWITCH A CHANNEL ON WITH A PWM VALUE                                                                           */
        /* -------------------------------------------------------------------------------------------------------------- */
        /* "PWM Channel on" gets 3 parameters:                                                                            */
        /*    • gReceivedData[1] is the OUTPUT # which can be: 3,5,6,9,10,11                                              */
        /*    • gReceivedData[2] is the PWM VALUE from 0-255                                                              */
        /*    • gReceivedData[3] is the ON TIME (in seconds) and is optional. This is the time the output will remain on, */
        /*      in seconds. Then it will switch off automatically. If "on time" is 0, it's ignored.                       */
        /* -------------------------------------------------------------------------------------------------------------- */

        // Filter out all the non PWM outputs
        if (rcvOutput <  3) {break;}
        if (rcvOutput == 4) {break;}
        if (rcvOutput == 7) {break;}
        if (rcvOutput == 8) {break;}
        if (rcvOutput > 11) {break;}

        // Set the output
        //Serial.println("PWM set.");
        analogWrite(rcvOutput, rcvValue1);

        // Check if there is timing to be done, if so, calculate and set the time to switch the output off
        if (rcvValue2 > 0) {gTimeArray[rcvOutput] = millis() + (rcvValue2 * 1000);}
      break;

      case 5: //PWM sweep
        // Filter out all the non PWM outputs
				if (rcvOutput <  3) {break;}
        if (rcvOutput == 4) {break;}
        if (rcvOutput == 7) {break;}
        if (rcvOutput == 8) {break;}
        if (rcvOutput > 11) {break;}

      break;

      case 6: //PWM sweep up
        // Filter out all the non PWM outputs
				if (rcvOutput <  3) {break;}
        if (rcvOutput == 4) {break;}
        if (rcvOutput == 7) {break;}
        if (rcvOutput == 8) {break;}
        if (rcvOutput > 11) {break;}

      break;

      case 7: //PWM sweep down
        // Filter out all the non PWM outputs
				if (rcvOutput <  3) {break;}
        if (rcvOutput == 4) {break;}
        if (rcvOutput == 7) {break;}
        if (rcvOutput == 8) {break;}
        if (rcvOutput > 11) {break;}

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
  checkBlinkAndSweep();

  /************************************************************************************************************************/
  /* Check if timed stuff has to be done and some output has to be switched off                                           */
  /************************************************************************************************************************/
  checkOffTimers();

}
