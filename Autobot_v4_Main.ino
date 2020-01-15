
/**************************************************************************************************************************/
/*                                                                                                                        */
/*                           _           _          _             _       ___  ___  ___                                   */
/*                          / \   _   _ | |_  ___  | |__    ___  | |_    |_ _||_ _||_ _|                                  */
/*                         / _ \ | | | || __|/ _ \ | '_ \  / _ \ | __|    | |  | |  | |                                   */
/*                        / ___ \| |_| || |_| (_) || |_) || (_) || |_     | |  | |  | |                                   */
/*                       /_/   \_\\__,_| \__|\___/ |_.__/  \___/  \__|   |___||___||___| v4.00 Due Edition                */
/*                                                                                                                        */
/*                                      Main sketch for the Due processor                                                 */
/*                                                                                                                        */
/*------------------------------------------------------------------------------------------------------------------------*/
/*                       3.30 Changed the MPU precision globally to 0,1°                                                  */
/*                       3.32 gAcumAngle thrown out in favor of gFinalAngle                                               */
/*                       3.40 Major changes to the program sequence, as per draw.io diagram                               */
/*                       3.41 Changed speeds when turning to full speed normally and 2 slow speeds in CheckTurnProgress() */
/*                       3.42 Touchbutton added                                                                           */
/*                       3.43 CheckDeviation() corrected                                                                  */
/*                       3.44 Lights control added                                                                        */
/*                       3.45 Light sweep on Halt added                                                                   */
/*                       3.46 Added TURN command, to go back, reversing all the way done                                  */
/*                       3.50 Change from I2C communication with the Gyro Board, to serial communication                  */
/**************************************************************************************************************************/

//#include <Wire.h>                   /* For using the I2C bus        */
//#include <LiquidCrystal.h>          /* For using the LCD display    */
#include <AccelStepper.h>           /* For using the Stepper Motors */

bool Step1 = false;
bool Step2 = false;
int Step = 0;


/*------------------------------------------------------------------------------------------------------------------------*
 * STEPPER STUFF                                                                                                          *
 *------------------------------------------------------------------------------------------------------------------------*/
  /* Create instance of the AccelStepper Class, specifying pins used for stepping and direction                                              */
  AccelStepper stepperFR(AccelStepper::DRIVER,28,29);  // Control type, stepping pin, direction pin
  AccelStepper stepperRR(AccelStepper::DRIVER,26,27);
  AccelStepper stepperFL(AccelStepper::DRIVER,24,25);
  AccelStepper stepperRL(AccelStepper::DRIVER,22,23);

/**************************************************************************************************************************
 *     __             _                                                                                                   *
 *     \ \   ___  ___| |_ _   _ _ __                                                                                      *
 *      \ \ / __|/ _ \ __| | | | '_ \                                                                                     *
 *      / / \__ \  __/ |_| |_| | |_) |                                                                                    *
 *     /_/  |___/\___|\__|\__,_| .__/ _____                                                                               *
 *                             |_|   |_____|                                                                              *
 **************************************************************************************************************************/
  void setup() {
    Serial.begin(115200);            /* start serial for output */
    moveSTOP();
    delay(1000);
    moveFORW(2000,1000);
}

void moveFORW(int distance, int theSpeed) {

  stepperFR.setMaxSpeed(theSpeed);
  stepperFR.setAcceleration(theSpeed);
  stepperFR.move(distance);

  stepperFL.setMaxSpeed(theSpeed);
  stepperFL.setAcceleration(theSpeed);
  stepperFL.move(distance * -1);

  stepperRR.setMaxSpeed(theSpeed);
  stepperRR.setAcceleration(theSpeed);
  stepperRR.move(distance);

  stepperRL.setMaxSpeed(theSpeed);
  stepperRL.setAcceleration(theSpeed);
  stepperRL.move(distance * -1);

}

void moveBACK(int distance, int theSpeed) {

  stepperFR.setMaxSpeed(theSpeed);
  stepperFR.setAcceleration(theSpeed);
  stepperFR.move(distance * -1);

  stepperFL.setMaxSpeed(theSpeed);
  stepperFL.setAcceleration(theSpeed);
  stepperFL.move(distance);

  stepperRR.setMaxSpeed(theSpeed);
  stepperRR.setAcceleration(theSpeed);
  stepperRR.move(distance * -1);

  stepperRL.setMaxSpeed(theSpeed);
  stepperRL.setAcceleration(theSpeed);
  stepperRL.move(distance);

}

void moveRIGH(int distance, int theSpeed) {

  stepperFR.setMaxSpeed(theSpeed);
  stepperFR.setAcceleration(theSpeed*2);
  stepperFR.move(distance * -1);

  stepperFL.setMaxSpeed(theSpeed);
  stepperFL.setAcceleration(theSpeed*2);
  stepperFL.move(distance * -1);

  stepperRR.setMaxSpeed(theSpeed);
  stepperRR.setAcceleration(theSpeed*2);
  stepperRR.move(distance * -1);

  stepperRL.setMaxSpeed(theSpeed);
  stepperRL.setAcceleration(theSpeed*2);
  stepperRL.move(distance * -1);

}

void moveLEFT(int distance, int theSpeed) {

  stepperFR.setMaxSpeed(theSpeed);
  stepperFR.setAcceleration(theSpeed);
  stepperFR.move(distance);

  stepperFL.setMaxSpeed(theSpeed);
  stepperFL.setAcceleration(theSpeed*2);
  stepperFL.move(distance);

  stepperRR.setMaxSpeed(theSpeed);
  stepperRR.setAcceleration(theSpeed*2);
  stepperRR.move(distance);

  stepperRL.setMaxSpeed(theSpeed);
  stepperRL.setAcceleration(theSpeed*2);
  stepperRL.move(distance);

}

void moveSTOP() {

  /* Stop motors dead, setting constant speed & eliminating acceleration */
  stepperFR.move(0); stepperFR.setSpeed(0); stepperFR.runSpeed();  /* .runSpeed() is the equivalent of .run() for constant speed */
  stepperFL.move(0); stepperFL.setSpeed(0); stepperFL.runSpeed();  /* .runSpeed() is the equivalent of .run() for constant speed */
  stepperRR.move(0); stepperRR.setSpeed(0); stepperRR.runSpeed();  /* .runSpeed() is the equivalent of .run() for constant speed */
  stepperRL.move(0); stepperRL.setSpeed(0); stepperRL.runSpeed();  /* .runSpeed() is the equivalent of .run() for constant speed */

}


/**************************************************************************************************************************
 *     __                    _                                                                                            *
 *     \ \   _ __ ___   __ _(_)_ __    _ __  _ __ ___   __ _ _ __ __ _ _ __ ___                                           *
 *      \ \ | '_ ` _ \ / _` | | '_ \  | '_ \| '__/ _ \ / _` | '__/ _` | '_ ` _ \                                          *
 *      / / | | | | | | (_| | | | | | | |_) | | | (_) | (_| | | | (_| | | | | | |                                         *
 *     /_/  |_| |_| |_|\__,_|_|_| |_| | .__/|_|  \___/ \__, |_|  \__,_|_| |_| |_| _____                                   *
 *                                    |_|              |___/                     |_____|                                  *
 **************************************************************************************************************************/
  void loop() {
   stepperFR.run();
   stepperFL.run();
   stepperRR.run();
   stepperRL.run();

/*   stepperFR.runSpeed();
   stepperFL.runSpeed();
   stepperRR.runSpeed();
   stepperRL.runSpeed();
*/
  Serial.print("FR:");Serial.print(stepperFR.distanceToGo());
  Serial.print("/FL:");Serial.print(stepperFL.distanceToGo());
  Serial.print("/RR:");Serial.print(stepperRR.distanceToGo());
  Serial.print("//RL:");Serial.println(stepperRL.distanceToGo());

/*
   if (stepperFR.distanceToGo() == 0) {
    switch (Step) {
      case 0:
        moveRIGH(500, 500);
        delay(2000);
        Step = Step + 1;
        break;
      case 1:
        moveLEFT(500, 500);
        delay(2000);
        Step = Step + 1;
        break;
      case 2:
        moveRIGH(2000, 700);
        delay(2000);
        Step = Step + 1;
        break;
      case 3:
        moveLEFT(500, 500);
        delay(2000);
        Step = Step + 1;
        break;
      case 4:
        moveSTOP();
        break;
      case 5:
        break;
      default:
      break;
    }
   }
*/
}
