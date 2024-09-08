void solveMaze()
{
#if defined(LEFTWALLFOLLOW) && !defined(RIGHTWALLFOLLOW) && !defined(DFS) && !defined(FLOODFILL)
    while (true)
    {
        wallFollow(RIGHT);
        // if (GLOBAL_X == 0 and GLOBAL_Y == 4)
        // {
        //     moveMotor(0, 0);
        //     // moveStraightAlignedToDistance(18, true, true);
        //     break;
        // }
    }
#elif !defined(LEFTWALLFOLLOW) && defined(RIGHTWALLFOLLOW) && !defined(DFS) && !defined(FLOODFILL)
    while (true)
        WallFollow(RIGHT);
#elif !defined(LEFTWALLFOLLOW) && !defined(RIGHTWALLFOLLOW) && defined(DFS) && !defined(FLOODFILL)
    dfs();
#elif !defined(LEFTWALLFOLLOW) && !defined(RIGHTWALLFOLLOW) && !defined(DFS) && defined(FLOODFILL)
    floodFill();
#else
#error "Please select a solving algorithm. Also, please make sure only one algorithm is enabled"
#endif
}

void update_coordinates()
{
    switch (HEADING)
    {
    case 0: // NORTH
        GLOBAL_Y++;
        Serial.print("Y++\t");
        break;
    case 1: // EAST
        GLOBAL_X++;
        Serial.print("X++\t");
        break;
    case 2: // SOUTH
        GLOBAL_Y--;
        Serial.print("Y--\t");
        break;
    case 3: // WEST
        GLOBAL_X--;
        Serial.print("X--\t");
        break;
    default:
        break;
    }
}

void printCoordinates(bool lw, bool rw)
{
    SerialDebug.print("<");
    SerialDebug.print(heading[HEADING]);
    SerialDebug.print(">\t(");
    SerialDebug.print(GLOBAL_X);
    SerialDebug.print(",");
    SerialDebug.print(GLOBAL_Y);
    SerialDebug.print(")\t");
    SerialDebug.print((lw) ? "L" : "-");
    SerialDebug.print(((fl + fr) / 2 > 50) ? "F" : "-");
    SerialDebug.println((rw) ? "R" : "-");
}

// unsigned long _tick = micros();
// void wallFollow(int wallToFollow)
// {
//     mpu.update();
//     updateProximitySensors();

//     if ((micros() - _tick) > 2000)
//     {
//         _tick = micros();

//         //////////////////////////////////////////////////////////////////////////////////
//         // Solving algorithm
//         //////////////////////////////////////////////////////////////////////////////////
//         bool coordinatesUpdated = false;
//         bool leftWall = (dl_filtered > leftTurnSense);
//         bool rightWall = (dr_filtered > rightTurnSense);
//         bool frontWall = ((fl + fr) / 2 > (leftTurnSense + rightTurnSense) / 2);
//         int8_t toTurn = 0;

//         moveStraightAligned();

//         int8_t _turn = detectTurn(leftWall, rightWall);

//         digitalWrite(LEFTLED, leftWall);
//         digitalWrite(RIGHTLED, rightWall);

//         float _len = mapFloat((float)encoderPos, 0, 61, 0, 10);

//         if (_len > 25)
//         {
//             resetEncoder();
//             update_coordinates();
//         }

//         if ((fl + fr) / 2 > 70)
//         {
//             _turn = 0;
//             SerialDebug.println("uturn");
//             turnBot(180, RIGHT);
// #ifdef BACKALIGN_UTURN
//             moveMotor(-50, -50);
//             delay(700);
//             moveMotor(0, 0);
//             updateWalls(false);
//             delay(200);
//             resetEncoder();
//             delay(5);
//             mpu.resetAngleZ();
// #endif
//         }
//         else if (_turn != 0)
//         {
//             moveStraightAlignedToDistanceForCurve(DISTANCE_TO_TRAVEL_AFTER_SIDEWALL_DETECT);
//             //Update the wall sensor parameters
//             updateProximitySensors();
//             leftWall = (dl_filtered > leftTurnSense);
//             rightWall = (dr_filtered > rightTurnSense);
//             frontWall = ((fl + fr) / 2 > (leftTurnSense + rightTurnSense) / 2);

//             bool sideWall = (_turn == LEFT) ? rightWall : leftWall;
//             bool followWall = (_turn == LEFT) ? (wallToFollow == LEFT) : (wallToFollow == RIGHT);

//             if (followWall)
//                 toTurn = _turn;
//             else if (frontWall)
//                 toTurn = sideWall ? _turn : -_turn;
//             else
//                 toTurn = sideWall ? 0 : -_turn;
//         }

//         switch (toTurn)
//         {
//         case LEFT:
//             coordinatesUpdated = true;
//             SerialDebug.println("left turn");
//             turnBot(90, LEFT, true);
//             updateWalls(false);
//             resetEncoder();
//             break;
//         case RIGHT:
//             coordinatesUpdated = true;
//             SerialDebug.println("right turn");
//             turnBot(90, RIGHT, true);
//             updateWalls(false);
//             resetEncoder();
//             break;
//         default:
//             break;
//         }
//     }
// }

bool printData = false;
bool updateHeading = false;

void wallFollow(int wallToFollow)
{
    mpu.update();
    updateProximitySensors();
    moveStraightAligned();

    bool leftWall = (dl_filtered > leftTurnSense);
    bool rightWall = (dr_filtered > rightTurnSense);

    float _len = mapFloat((float)encoderPos, 0, 57, 0, 10);
    int _distance = (int)(_len / 25);

    if (updateHeading)
    {
        // update_heading();
        updateHeading = false;
    }
    if (printData)
    {
        printData = false;
        SerialDebug.print((int)_distance);
        SerialDebug.print("\t");
        SerialDebug.print("<");
        SerialDebug.print(heading[HEADING]);
        SerialDebug.print(">\t(");
        SerialDebug.print(GLOBAL_X);
        SerialDebug.print(",");
        SerialDebug.print(GLOBAL_Y);
        SerialDebug.print(")\t");
        SerialDebug.print((leftWall) ? "L" : "-");
        SerialDebug.print(((fl + fr) / 2 > 50) ? "F" : "-");
        SerialDebug.println((rightWall) ? "R" : "-");
    }

    if (_distance != prevTile)
    {
        prevTile = _distance;
        updateHeading = true;
        printData = true;
    }

    digitalWrite(LEFTLED, leftWall);
    digitalWrite(RIGHTLED, rightWall);

    int turnDetected = detectTurn(leftWall, rightWall);

    int toTurn = 0;
    int turnMode = 0;

    if ((fl + fr) / 2 > 30)
    {
        turnMode = FRONTWALL;
        leftWall = (dl_filtered > WallSenseDuringTurn);
        rightWall = (dr_filtered > WallSenseDuringTurn);
        if (leftWall and rightWall)
            toTurn = UTURN;
        else if (!leftWall and rightWall)
            toTurn = LEFT;
        else if (leftWall and !rightWall)
            toTurn = RIGHT;
        else if (wallToFollow == LEFT)
            toTurn = LEFT;
        else
            toTurn = RIGHT;
    }
    else if (turnDetected != 0)
    {
        if (wallToFollow == LEFT and turnDetected == LEFT) // for left wall follow
        {
            toTurn = LEFT;
            turnMode = ADJECENTWALL;
        }
        else if (wallToFollow == RIGHT and turnDetected == RIGHT)
        {
            toTurn = RIGHT;
            turnMode = ADJECENTWALL;
        }
    }

    if (turnMode == ADJECENTWALL)
        moveStraightAlignedToDistanceForCurve(DISTANCE_TO_TRAVEL_AFTER_SIDEWALL_DETECT);

    switch (toTurn)
    {
    case LEFT:
        updateHeading = true;
        printData = true;
        turnBot(90, LEFT, true);
        updateWalls(false);
        resetEncoder();
        break;
    case RIGHT:
        updateHeading = true;
        printData = true;
        turnBot(90, RIGHT, true);
        updateWalls(false);
        resetEncoder();
        break;
    case UTURN:
        Serial.print("UTURN");
        moveStraightAlignedToDistance(8, true, true);
        turnBot(180, RIGHT);

#ifdef BACKALIGN_UTURN
        moveMotor(-50, -50);
        delay(700);
        moveMotor(0, 0);
        updateWalls(false);
        delay(200);
        resetEncoder();
        delay(5);
        mpu.resetAngleZ();
#endif
        break;
    default:
        break;
    }
    // delayWithSensorUpdate(5);
}

void dfs() // TODO
{
    while (1)
    {
    }
}

void floodFill() // TODO
{
    while (1)
    {
    }
}