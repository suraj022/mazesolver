#include "header.h"

void setup()
{
  /* Initialize Serial module */
  SerialDebug.begin(115200);

  /* Set motor outputs */
  setMotorIOs();
  /* Initialize I2C module */
  beginWire();

  /* Initialize MPU6050 module */
  beginMpu();

  /* Set pin 7, 13 as output */
  pinMode(BUZZPIN, OUTPUT);
  pinMode(LED13, OUTPUT);

  pinMode(LEFTLED, OUTPUT);
  pinMode(RIGHTLED, OUTPUT);

  pinMode(enc1, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);

  moveMotor(0, 0);

  /* Scan I2C devices (uncomment to check connected I2C device addresses) */
  // testI2C();

  /* Initializes all proximity sensors */
  initAPDS(FL);
  initAPDS(DL);
  initAPDS(FR);
  initAPDS(DR);

  /* let the bot rest and wait for user input from proximity sensor > diagonal left */
  waitForUserInput1();

  /* Calculate gyroscope offsets and wait for user input from proximity sensor > diagonal right */
  mpu.calcOffsets(true, true); // gyro and accel
  mpu.resetAngleZ();
  waitForUserInput2();

  encoderPos = 0;
  enableISR();

  SerialDebug.println("Start");

  /* test the rotation accuracy using diagonal sensors. comment to disable testing. */
  // testRotation();

  /* Start motors */
  for (int i = 0; i < MAXSPEED_move; i++)
  {
    mpu.update();
    int gyroerror = mpu.getAngleZ() * 2;
    moveMotor(i + gyroerror, i - gyroerror);
    delayMicroseconds(250);
  }

  /* Run main algorithm
  Note: function is blocking*/
  solveMaze(); 

  /* Stop the motors*/
  moveMotor(0,0);

  /* indicate finish state by blinking both leds continously. */
  while (1)
  {
    state = !state;
    digitalWrite(LEFTLED, state);
    digitalWrite(RIGHTLED, state);
    delay(400);
  }
}

void loop()
{
  /* Nothing to do here */
  delay(1);
}
