// Note that the scout robot has to get the distance until the end of the wall,
// then again from the end of the wall to the balloon (such that there is a reference point from where it hit the wall)

void drive (int a , int c) //Sets motor powers of the two motors
{
	motor [motorA] = a; // INNER (RIGHT)
	motor [motorC] = c; // OUTER (LEFT)
}

//Returns converted distance in CENTIMETERS
float distance (int clickNum )
{
	return ( 3.5 * PI * clickNum / 360.0 );
}

float wallDistance ( int speed ) //Distance to the wall
{
	nMotorEncoder [motorA] = 0;
	drive ( 75 , 75 );
	while ( SensorValue (S3) == 0 ){}
	return distance ( nMotorEncoder [motorA] );
}

// Turns clockwise for the given milliseconds, and backs up if true
void turn ( bool back , int time )
{
	if ( back == true )
	{
		clearTimer (T1);
		drive ( -30 , -30 ) ;
		while ( time1 [T1]  < 450 ){}
	}

	clearTimer (T1);
	drive ( 50 , -50 ) ;
	while ( time1[T1]  < time ){}
	drive ( 0 , 0 );
}

/* This is the auto adjusting function that allows the scout robot effectively run parallel to the wall when it is measuring
distance from the point it hit to the end of the wall, then from the end of the wall to the start of the balloon, then to the
end of the balloon, and as it goes back and checks for whether or not the balloon was successfully popped

Parameters in this function allow it to be coded once, but called for a variety of different implementations.

speed is used to set the speed at which the motors turn, calling the drive function

s1LessThan is used to set a value that the horizontal facing sensor has to see less than for the auto adjust to keep running

LessThan is a value that the vertical sensor must see less than for auto adjust to keep running

GreaterThan is a value that the vertical sensor must see more than for auto adjust to keep running

x and y are variables passed by reference that relay the x and y readings of the robot for the balloon coordinates

change is the value that the outer motor changes the speed by every time it needs to adjust its path

under is a boolean to tell the function if the robot is below the balloon, and if it is it must sense 255 two times in a row
to say that it is not under the balloon anymore ( to accomodate for the sensors errors when it accidentally reads 255
*/

void autoAdjust ( int speed , int s1LessThan , int LessThan , int GreaterThan , float & x , int & y , int change , bool under)
{
	int dist = -1 ;
//Initializes the original distance from the wall, making sure it's not 255
	while ( dist == -1 )
	{
		if ( SensorValue (S1) != 255 )
			dist = SensorValue (S1);
	}

  bool quit = false; //Boolean used to quit the auto adjust loop if it is under the balloon and sees 255 more than once in a row
  int previous = 0 ;

	drive ( speed , speed ) ;
	nMotorEncoder [motorA] = 0 ;

	while ( SensorValue (S1) < s1LessThan && SensorValue (S4) >  GreaterThan && SensorValue (S4) < LessThan && distance (nMotorEncoder[motorA] ) < 150  && quit == false) //Can be set to a value different than 30 cm
	{
		if ( under == true ) //Needs 2 values in a row of 255 if its under the balloon to exit the loop
		{
			if ( previous==255 && SensorValue (S4)==255 )
				quit = true;

			else if ( previous!=255 && SensorValue (S4)==255 )
				previous = 255;

			else if ( previous!=255 && SensorValue (S4)!=255 )
				previous = 0 ;
		}

//Pivots the robot away from the wall if the horizontal sensor senses a distance less than the previous one
		if ( SensorValue (S1) < dist )
		{
			drive ( speed , speed - change );
			wait1Msec (300);
			dist = SensorValue (S1);
		}

//Pivots the robot toward the wall if the horizontal sensor senses a distance greater than the previous one
		else if (SensorValue(S1) > dist && SensorValue(S1)!=255)
		{
			drive ( speed , speed + change );
			wait1Msec (300);
			dist = SensorValue (S1);
		}

//Delays area added throughout the function to prevent an overload, and give time for the adjustment to make a change
		drive ( speed, speed ) ;
		wait1Msec ( 100 ) ;
	}
	drive ( 0 , 0 ) ;
//Sets y value currently seen by the vertical sensor
	y = SensorValue (S4);

//Sets x value as what the motors have travelled since the function was ran
    x = fabs ( distance ( nMotorEncoder [motorA] ) );
}

// Written by Ion Buzdugan
//Drives to the end of the wall and passes the distance it drove
void driveEndWall ( int speed , float & x , int & y)
{
	autoAdjust ( speed , 30 , 300 , -1 , x , y , -15 , false);
}

void findBalloon ( float & x , int & y)
{
	nMotorEncoder [motorA]= 0;
	drive ( 10 , 10 ) ;
	wait1Msec ( 2500 );
	float x1 = ( distance ( nMotorEncoder [motorA] ) );
// Until finds first end of base
	autoAdjust ( 10 , 300 , 256 , 254 , x , y , 15 , false);
	x1 += x;
	autoAdjust ( 50 , 300 , 256 , 0 , x , y , 10 , true );
	float forward = x;
	autoAdjust ( -10 , 300 , 256 , 254 , x , y , -10 , false);
	float x2 = x;
 	x =  x1 + (  forward / 2 );
}

bool checkPop ()
{
	clearTimer (T1);
	drive ( 50 , 50 ) ;
	while ( time1[T1] < 3000 && SensorValue (S4) == 255 ) {}
	drive ( 0 , 0 );
	if ( SensorValue (S4) == 255 )
		return true; //returns that it is popped if upward sonar does not sense anything at the previous position of the robot
	else
		return false;
}

//Waits for any button to be pressed and released
void waitPress ()
{
	while ( nNxtButtonPressed == -1 ){}
	while ( nNxtButtonPressed != -1 ){}
}

// Clears mailbox as long as there is something in it
void clearMsg ()
{
	while (bQueuedMsgAvailable() )
	{
		ClearMessage ();
		int k = message ;
		wait1Msec ( 100 );
	}
}

task main()
{
	float x = 0 , dist = 0 , fromMiddle = 0 ;
	int y = 0 ;
	SensorType (S1) = sensorSONAR; //sideways facing sonar
	SensorType (S4) = sensorSONAR; //upwards facing sonar
	SensorType (S3) = sensorTouch;

	clearMsg ();

	// Wait for message to start
	while(!bQueuedMsgAvailable())
		wait1Msec(75);

	clearMsg ();

	// Determines location of the balloon
	dist  = wallDistance (50) + 25;
	turn (true , 1000);
	driveEndWall ( -50 , x , y ) ;
	fromMiddle = x ;
	findBalloon ( x , y ) ;

	while ( SensorValue (S4) == 255 ){}
	y = SensorValue (S4) + 12 ;

	// Sends coordinates of balloon to launcher robot
	for ( int i = 0 ; i < 5 ; i ++ )
	{
		sendMessageWithParm(fromMiddle-x, y, dist);
		wait1Msec ( 40 );
	}

	// Turns and checks if range clear, returns state to launcher robot
	turn ( false , 2000 );
	int check = 0;

	do
	{
		while(!bQueuedMsgAvailable())
			wait1Msec(75);
		while (bQueuedMsgAvailable())
		{
			check = message;
			ClearMessage();
		}
		if (check == 2)
		{
			for ( int i = 0 ; i < 5 ; i ++ )
			{
				sendMessage((SensorValue (S1)>dist-20 || SensorValue(S1) == 255)+1);
				wait1Msec ( 50 ) ;
			}
		}
	} while (check !=3) ;

	// Move to end of wall
	turn (false, 1800);

	clearTimer (T1);
	drive ( -50 , -50 ) ;
	while ( time1[T1] < 3000 ) {}
	drive ( 0 , 0 );

	clearMsg ();
	//Waits for message to go check for a balloon that's popped
	while(!bQueuedMsgAvailable())
		wait1Msec(75);
	clearMsg ();


// Checks if balloon is popped and sends result to launcher
	if ( checkPop () == true )
	{
		for ( int i = 0 ; i < 5 ; i ++ )
			sendMessage(4);
	}
	else
	{
		for ( int i = 0 ; i < 5 ; i ++ )
			sendMessage(5);
	}
	waitPress ();
}
