/***************************************/
/*   Group C                           */
/*   3/22/2023                         */
/*   Checkpoint 5                      */
/***************************************/


#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>
#include <stdio.h>
#include <FEHservo.h>


// Motor Inputs
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_2);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);
FEHServo arm_servo(FEHServo::Servo7);


// Bumper Switch Inputs
DigitalInputPin frontSwitchLeft(FEHIO::P1_2);
DigitalInputPin frontSwitchRight(FEHIO::P1_4);


// CdS Cell Input
AnalogInputPin sensor(FEHIO::P1_0);


// Line Following Inputs
AnalogInputPin leftLineF(FEHIO::P2_0); // BLUES
AnalogInputPin centerLineF(FEHIO::P2_3);  // GREYS
AnalogInputPin rightLineF(FEHIO::P2_7);  // REDS


//Define cool constants
#define LEFT_TURN_PERCENT 40
#define RIGHT_TURN_PERCENT 40
#define LEFT_RAMPING_SPEED 50
#define RIGHT_RAMPING_SPEED 50


void NO() // This is a permanant while loop that stops all code
{
    LCD.Clear(SCARLET);
    while(true)
    {
        LCD.WriteLine("NO");
    }
}


void drive(int FoB, int percent, float inches)
{
    // drive(0) will drive backwards
    // drive(1) will drive forwards
    // Added inches to counter converter, so
    // we will only need to enter the inches we need to travel
    // this can be extended to RPS units once we know the conversion


    // Declare counts
    float counts;


    // convert inches to tic counts
    counts = (inches*318)/11;


    // Set forward (1) or backward (0)
    if(FoB==1)
    {
        percent = percent;
    }
    if(FoB==0)
    {
        percent = -percent;
    }


    // Set counts to zero
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    //Set both motors to percent speed WILL NEED ADJUSTMENT FOR MOTORS
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2 < counts);


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegree(int direction, int degree) //This function turns the robot a set degree using one of its wheels as a pivot point
{
    // turn(0) turns to the left (counterclockwise)
    // turn(1) turns to the right (clockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=7;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor power full speed forward
        right_motor.SetPercent(RIGHT_TURN_PERCENT);
        //left_motor.SetPercent(-LEFT_TURN_PERCENT);        
    }
    else
    {
        // set right motor power full speed backward
        left_motor.SetPercent(LEFT_TURN_PERCENT);
        //right_motor.SetPercent(-RIGHT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while((left_encoder.Counts() + right_encoder.Counts())<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegreeCenter(int direction, int degree) //This function turns the robot a set degree using one of its wheels as a pivot point
{
    // turn(0) turns to the left (counterclockwise)
    // turn(1) turns to the right (clockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=3.5;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor power full speed forward
        right_motor.SetPercent(RIGHT_TURN_PERCENT);
        left_motor.SetPercent(-LEFT_TURN_PERCENT);        
    }
    else
    {
        // set right motor power full speed backward
        left_motor.SetPercent(LEFT_TURN_PERCENT);
        right_motor.SetPercent(-RIGHT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while(((left_encoder.Counts() + right_encoder.Counts())/2)<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegreeBackwards(int direction, int degree) //This function turns the robot a set degree backwards using one of its wheels as a pivot point
{
    // turn(0) turns to the left (clockwise)
    // turn(1) turns to the right (counterclockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=7;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor
        right_motor.SetPercent(-RIGHT_TURN_PERCENT);        
    }
    else
    {
        // set left motor power
        left_motor.SetPercent(-LEFT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while((left_encoder.Counts() + right_encoder.Counts())<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void testRightMotor(int percent)
{
    right_motor.SetPercent(percent);
}


void testLeftMotor(int percent)
{
    left_motor.SetPercent(percent);
}


void whitelinefollowing()
{
    // to be used to follow a line for either time or distance
    float L, R, C;
    // declare L,R,M as left, right and middle line sensors
    //Variables that display values for analogue inputs
    L=leftLineF.Value();
    C=centerLineF.Value();
    R=rightLineF.Value();


    //Line following algorithm
    //Determine case
    if (L<2.4 && C>0.45 && R>2.4)
    {
        //Need to go left
        left_motor.SetPercent(0);
        right_motor.SetPercent(30);
        Sleep(0.5);
        left_motor.SetPercent(20);
        right_motor.SetPercent(20);
    }
    else if (L>2.4 && C>0.45 && R<2.4)
    {
        //Need to go Right
        left_motor.SetPercent(30);
        right_motor.SetPercent(0);
        Sleep(0.5);
        left_motor.SetPercent(20);
        right_motor.SetPercent(20);
    }
    else
    {
        //Need to continue straight
        left_motor.SetPercent(15);
        right_motor.SetPercent(15);
        Sleep(0.5);
    }
}


void yellowlinefollowing()
{
    // to be used to follow a line for either time or distance
    float L, R, C;
    // declare L,R,M as left, right and middle line sensors
    //Variables that display values for analogue inputs
    L=leftLineF.Value();
    C=centerLineF.Value();
    R=rightLineF.Value();


    //Line following algorithm
    //Determine case
    if (L<2.0 && C<2.0 && R>2.0)
    {
        //Need to go left
        left_motor.SetPercent(0);
        right_motor.SetPercent(30);
        Sleep(0.5);
        left_motor.SetPercent(15);
        right_motor.SetPercent(15);
        Sleep(0.5);
    }
    else if (L>2.0 && C<2.0 && R<2.0)
    {
        //Need to go Right
        left_motor.SetPercent(30);
        right_motor.SetPercent(0);
        Sleep(0.5);
        left_motor.SetPercent(15);
        right_motor.SetPercent(15);
        Sleep(0.5);
    }
    else
    {
        //Need to continue straight
        left_motor.SetPercent(15);
        right_motor.SetPercent(15);
        Sleep(0.5);
    }
}


void displaySensorValues() //This function records and displays our values from the Cds cells and line following sensors, for testing purposes
{
    //Declare variables
    float CdS, leftLine, centerLine, rightLine;
    char s[30];


    while(true)
    {
        //Set variables equal to their respective values
        CdS=sensor.Value();
        leftLine=leftLineF.Value();
        centerLine=centerLineF.Value();
        rightLine=rightLineF.Value();


        //Clear the screen and print the values
        LCD.Clear();
        sprintf(s, "%f %f %f %f", CdS,leftLine,centerLine,rightLine);
        LCD.WriteLine(s);


        //Sleep so you got time to read
        Sleep(0.2);
    }
}


void bumperTest() //This function tests the bumper plate switches by displaying their values to the screen
{
    //Declare variables and clear screen
    int leftSwitch, rightSwitch;
    char s[30];
    LCD.Clear();


    while(true) //Take values from the switches and then print them to the screen continuously
    {
        leftSwitch=frontSwitchLeft.Value();
        rightSwitch=frontSwitchRight.Value();


        sprintf(s, "Left = %i and Right = %i", leftSwitch, rightSwitch);
        LCD.WriteLine(s);
    }
}


int determineColor() //This function determines the color of the light for the boarding pass and then moves to the correct light and displays it
{
    //Read the value of the CdS sensor
    float CdS;
    int return_color;


    //Sleep to get good reading
    Sleep(1.0);


    //read the value
    CdS=sensor.Value();


    //For Red
    if(CdS < 0.40)
    {
        //Make screen red
        LCD.Clear(RED);
        return_color=0;
    }
    //For Blue
    else if(CdS > 0.55)
    {
        //Make screen blue
        LCD.Clear(BLUE);  
        return_color=1;
    }
    else //error
    {
        return_color=2;
    }
    return return_color;
}


void goToWall(int percent, float T) //This function drives straight until we hit a wall, or cuts out after a certain time
{
    LCD.Clear();
    LCD.WriteLine("DRIVING to waLL");
    float startTime=TimeNow();
    while((frontSwitchLeft.Value() || frontSwitchRight.Value()) && ((TimeNow()-startTime)<T))
    {
        //Drive forward with percent power
        left_motor.SetPercent(percent+1);
        right_motor.SetPercent(percent);


        // Print ellapsed time status
        LCD.Clear();
        LCD.WriteLine("wall stuff");
        LCD.WriteLine(TimeNow()-startTime);
    }


    //Stop Motors
    left_motor.Stop();
    right_motor.Stop();


    //Kowalski, status report
    LCD.Clear();
    LCD.WriteLine("Skipper, I appear to have hit a wall");


    //Sleep for a sec
    Sleep(0.5);
}


void searchForLight() //This function drives forward until the robot detects the light
{
    int CdS;


    while(CdS=sensor.Value()>2.5)
    {
        //Drive until you find the light
        left_motor.SetPercent(20);
        right_motor.SetPercent(20);
    }


    //Stop motors
    left_motor.Stop();
    right_motor.Stop();
}


void initializeServo()
{
    // set servo min and max
    arm_servo.SetMin(500);
    arm_servo.SetMax(2500);
}


int correctLever() // This function gets the correct lever from RPS and saves it to an integer
{
    int cLever; //Variable we will return


    // Get the Lever from RPS
    cLever = RPS.GetCorrectLever();


    return cLever;


    // 0 is lever A
    // 1 is lever A1
    // 2 is lever B
}


void hitLever(int cLever) // This function hits the correct lever
{
    // Take a breather before the lever
    LCD.Clear();
    LCD.WriteLine("About to hit lever");
    Sleep(1.0);


    // Cases for each lever
    if(cLever == 0) // Lever A
    {
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever A");


        // Turn to the left a bit
        turnDegree(0,25);
        Sleep(2.0);


        // Lower Arm to flip lever down
        arm_servo.SetDegree(60);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever A Down, waiting 5 seconds");
        Sleep(5.0);


        // Drive Back then forward to let arm under the lever
        drive(0, 30, 3);
        arm_servo.SetDegree(0);
        drive(1, 30, 3);
       
        // Raise Arm to lift lever
        arm_servo.SetDegree(60);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever A Back Up!");
    }
    else if(cLever == 1) // Lever A1
    {
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever A1");


        //No need to turn, drive forward a bit and just flip the lever now
        drive(1,30,1);
        arm_servo.SetDegree(60);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever A1 Down, waiting 5 seconds");
        Sleep(5.0);


        // Drive Back then forward to let arm under the lever
        drive(0, 30, 3);
        arm_servo.SetDegree(0);
        drive(1, 30, 3);
       
        // Raise Arm to lift lever
        arm_servo.SetDegree(60);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever A1 Back Up!");
    }
    else if(cLever == 2) //Lever B
    {
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever B");


        // Turn to the right a bit
        // 1 is clockwise
        turnDegree(1,40);
        Sleep(2.0);


        // Flip Lever Down
        arm_servo.SetDegree(25);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever B Down, waiting 5 seconds");
        Sleep(5.0);


        // Drive Back then forward to let arm under the lever
        drive(0, 30, 3);
        arm_servo.SetDegree(0);
        drive(1, 30, 3);
       
        // Raise Arm to lift lever
        arm_servo.SetDegree(60);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever B Back Up!");
    }
    else //Error
    {
       LCD.Clear(); // Status Report
       LCD.WriteLine("Invalid Lever");
    }
}


void rpsInfo() // Displays RPS Data
{
    LCD.Clear();
    LCD.Write(RPS.X());
    LCD.Write(RPS.Y());
    LCD.Write(RPS.Heading());
    LCD.Write(RPS.GetCorrectLever());
    Sleep(2.0);
}


void initialize() // This contains all the initialization code for the robot
{
    // Set Min and Max for Servo and initialize RPS
    initializeServo();
    arm_servo.SetDegree(180);
    RPS.InitializeTouchMenu();
}


void CDSstart() // This contains the code to start with the CDS cell input
{
    //Declare CdS variable
    float CdS=sensor.Value();


    // code for start with light
    LCD.Clear();


    // read the value of sensor
    while(CdS>2)
    {
        CdS=sensor.Value();
        LCD.Clear(); // Status report
        LCD.WriteLine("Waiting for start light");
        Sleep(2.0);
    }
    LCD.WriteLine("Starting!"); //Status report
}


void luggage() // This function deposits the luggage in the big ol box (idk if we're doing high or low yet)
{
    // Drop Servo
    arm_servo.SetDegree(100);


    // Sleep and status
    LCD.Clear();
    LCD.WriteLine("The package has been delivered");
    Sleep(0.5);


    // Drive backwards a bit
    drive(0,30,4);
}


int main(void)
{  
    // Startup
    initialize();
    CDSstart();


    // Drive forward 9 inches
    drive(1,30,9);


    // Turn a bit to the right
    turnDegree(1,50);


    // go to the wall
    goToWall(30, 2.0);


    // Deposit Luggage
    luggage();


    // turn to face final button
    turnDegreeBackwards(1, 50);


    // Drive towards end button
    drive(0, 50, 20);
}

