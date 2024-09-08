
void setMotorIOs()
{
  // Set motor OUTPUTs
  pinMode(LMF, OUTPUT);
  pinMode(LMR, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(RMR, OUTPUT);
  pinMode(PWMR, OUTPUT);

  // Set digital LOW to all OUTPUTs
  moveMotor(0, 0);

  delay(10);
}

void moveMotor(int inL, int inR)
{
  inL = (float)inL * 0.985;
  int leftpwm = abs(inL);
  int rightpwm = abs(inR);

  leftpwm = constrain(leftpwm, 0, 255);
  rightpwm = constrain(rightpwm, 0, 255);

  bool lmf = (inL > 0) ? 1 : 0;
  bool lmr = (inL > 0) ? 0 : 1;
  bool rmf = (inR > 0) ? 1 : 0;
  bool rmr = (inR > 0) ? 0 : 1;

  analogWrite(PWML, leftpwm);
  digitalWrite(LMF, lmf);
  digitalWrite(LMR, lmr);

  analogWrite(PWMR, rightpwm);
  digitalWrite(RMF, rmf);
  digitalWrite(RMR, rmr);
}

void testRotation()
{
  while (1)
  {
    updateProximitySensors();
    if (dl > 90)
    {
      Serial.println("Turn left");
      turnBot(90, LEFT);
      delay(500);
    }
    if (dr > 90)
    {
      Serial.println("Turn right");
      turnBot(90, RIGHT);
      delay(500);
    }
  }
}

void frontWallAlign()
{
  moveMotor(0, 0);
  unsigned long timer = 0;
  unsigned long timerExit = 0;
  bool enableExitTimer = false;
  float _p = 0;
  float _d = 0;
  float _pe = 0;

  mpu.resetAngleZ();
  delay(50);
  unsigned long MAX_TIMER = millis();
  while (true)
  {
    mpu.update();
    updateProximitySensors();
    if ((millis() - timer) > 10)
    {
      int error = (fl_filtered - fr_filtered) - (LEFTWALLOFFSET - RIGHTWALLOFFSET); // CALCULATE
      if (abs(error) <= 1 and !enableExitTimer)
      {
        enableExitTimer = true;
        timerExit = millis();
      }
      float _ce = error - _pe;
      _pe = error;
      _p = error * kp_align;
      _d = _ce * kd_align;
      int drive = _p + _d;
      drive = constrain(drive, -MAXSPEED_align, MAXSPEED_align);
      moveMotor(-drive, drive);
      if (enableExitTimer)
      {
        if ((millis() - timerExit) > 100)
          break;
      }
      if ((millis() - MAX_TIMER) > 800)
        break;
      timer = millis();
    }
  }
  moveMotor(0, 0);
}

void turnBot(int degrees, int direction)
{
  turnBot(degrees, direction, false);
}

void turnBot(int degrees, int direction, int _curve)
{
  if (!_curve)
    moveMotor(0, 0);
  unsigned long timer = 0;
  unsigned long timerExit = 0;
  bool enableExitTimer = false;
  float _p = 0;
  float _d = 0;
  float _pe = 0;

  // int _degrees = (float)degrees * .96;
  mpu.resetAngleZ();
  delay(5);
  unsigned long MAX_TIMER = millis();
  while (true)
  {
    mpu.update();
    updateProximitySensors();
    if ((millis() - timer) > 10)
    {
      int error = abs(degrees) - abs(mpu.getAngleZ());
      if (error <= 1 and !enableExitTimer)
      {
        enableExitTimer = true;
        timerExit = millis();
        if (_curve)
        {
          moveMotor(BASESPEED_move, BASESPEED_move);
          break;
        }
      }
      float _ce = error - _pe;
      _pe = error;

      _p = error * kp_turn;
      _d = _ce * kd_turn;

      if (_curve)
        _p *= 1.55;
      int drive = _p + _d;
      float turning = MAXSPEED_turning;
      if (_curve)
        turning *= 1.5;
      if (turning > 255)
        turning = 255;
      if (drive > turning)
        drive = turning;
      if (drive < -turning)
        drive = -turning;
      moveMotor((direction * drive < 0 and _curve) ? -direction * drive / 8 : direction * drive, (-direction * drive < 0 and _curve) ? direction * drive / 8 : -direction * drive);
      if (enableExitTimer)
      {
        if ((millis() - timerExit) > 100)
          break;
      }
      if ((millis() - MAX_TIMER) > 800)
        break;
      timer = millis();
    }
  }

  if (degrees == 90 and direction == LEFT)
    HEADING = (HEADING + 3) % 4;
  else if (degrees == 90 and direction == RIGHT)
    HEADING = (HEADING + 1) % 4;
  else if (degrees == 180)
    HEADING = (HEADING + 2) % 4;
  prevTile = 0;
  if (!_curve)
    moveMotor(0, 0);
  mpu.resetAngleZ();
  resetEncoder();
}

void moveStraightAligned()
{
  int leftWallValue = dl_filtered;
  int rightWallValue = dr_filtered;
  bool leftWall = (leftWallValue > wallSense);
  bool rightWall = (rightWallValue > wallSense);
  int error = 0;
  int gyroerror = mpu.getAngleZ() * kp_gyroAssist;
  if (leftWall and rightWall)
  {
    error = leftWallValue - rightWallValue;
  }
  else if (leftWall and !rightWall)
  {
    error = leftWallValue - leftWallOffset;
  }
  else if (!leftWall and rightWall)
  {
    error = leftWallOffset - rightWallValue;
  }
  error = constrain(error, -MAX_error, MAX_error);
  int ce_move = error - pe_move;
  pe_move = error;

  p_move = error * kp_move;
  d_move = ce_move * kd_move;
  int drive = p_move + d_move;
  if (drive > MAXSPEED_move)
    drive = MAXSPEED_move;
  if (drive < -MAXSPEED_move)
    drive = -MAXSPEED_move;
  moveMotor(BASESPEED_move + drive + gyroerror, BASESPEED_move - drive - gyroerror);
}

void moveStraightAlignedToDistanceForCurve(float _distance)
{
  resetEncoder();
  mpu.resetAngleZ();
  while (true)
  {
    mpu.update();
    updateProximitySensors();
    float _len = mapFloat((float)encoderPos, 0, encoder_scaling, 0, 10);
    float gyroerror = mpu.getAngleZ() * kp_gyroAssist;
    if (_len > _distance)
      break;
    moveMotor((int)(BASESPEED_move + gyroerror), (int)(BASESPEED_move - gyroerror));
  }
}

void moveStraightAlignedToDistance(float _distance, bool wasRunning, bool _stopAtFrontWall)
{
  resetEncoder();
  mpu.resetAngleZ();
  float p_move_distance = 0;
  float d_move_distance = 0;
  float pe_move_distance = 0;
  if (!wasRunning)
  {
    for (int i = 0; i < MAXSPEED_move_distance; i++)
    {
      moveMotor(i, i);
      delayMicroseconds(1400);
    }
  }
  while (true)
  {
    mpu.update();
    updateProximitySensors();
    float _len = mapFloat((float)encoderPos, 0, encoder_scaling, 0, 10);
    float error = _distance - _len;
    float gyroerror = mpu.getAngleZ() * kp_gyroAssist;
    float ce_move_distance = error - pe_move_distance;
    pe_move_distance = error;
    p_move_distance = error * kp_move_distance;
    d_move_distance = ce_move_distance * kd_move_distance;
    float drive = p_move_distance + d_move_distance;
    drive = constrain(drive, MINSPEED_move_distance, MAXSPEED_move_distance);
    moveMotor((int)(drive + gyroerror), (int)(drive - gyroerror));
    if (error <= 0)
    {
      _beep();
      break;
    }
  }
  moveMotor(0, 0);
  delay(30);
}