// Note: discarded messages are purposely stored in unused
// integers as otherwise they do not clear

// Uses the touch sensor to reset the vertical angle of the launcher
void resetVertAngle()
{
	// Raises launcher until touch sensor not pressed
	motor[motorC] = -25;
	while(SensorValue[S2] == 1) {}
	nMotorEncoder[motorC] = 0;
	while(nMotorEncoder[motorC] > -90) {}
	// Lowers launcher until touch sensor pressed
	motor[motorC] = 15;
	while(SensorValue[S2] == 0) {}
	// Raises launcher slightly so not resting on touch sensor
	motor[motorC] = -15;
	nMotorEncoder[motorC] = 0;
	while(nMotorEncoder[motorC] > -45) {}
	motor[motorC] = 0;
}

// Solves for the required vertical angle based on balloon coordinates
float kinsolve (float xzdist, float ydist)
{
	float a = (9.807*xzdist*xzdist)/(-2*1150*1150);
	float b = xzdist, c = a - ydist;
	float discriminant = b*b - 4*a*c;
	if (discriminant < 0 || a == 0)
	{
		// Returns -1 if out of range
		displayString(0, “Out of Range”);
		return -1.0;
	}
	else
	{
		float angle = atan((-b + sqrt(discriminant))/(2.0*a));
		// Returns angle in radians
		return angle;
	}
}

// Angles launcher vertically based on balloon coordinates
void angleArmVert (float xzdist, float y)
{
	float angle = (kinsolve(xzdist, y) * (180.0/PI)) - 12.8;
	// Calculates motor rotations required to angle launcher
	int mRotations = -(angle / (-0.00018*xzdist+0.075));
	// Rotates launcher to angle
	nMotorEncoder[motorC] = 0;
	motor[motorC] = -25;
	while (nMotorEncoder[motorC] > mRotations) {}
	motor[motorC] = 0;
}

// Calculates required horizontal angle based on balloon coordinates
float findHorzAngle (float x, float z)
{
	float angle = atan(x/ z) * (180.0/PI);
	// Returns angle in degrees
	return angle;
}

// Angles launcher horizontally based on balloon coordinates
int angleArmHorz (float x, float z)
{
	float horzAngle = findHorzAngle(x,z);
	// Calculates motor rotations required to angle launcher
	int mRotations = -(round(horzAngle / (24.0/56.0)));
	nMotorEncoder[motorA] = 0;
	// Determines direction required to angle launcher
	if (mRotations > 0)
	{
		// slightly adjusts to account for inaccuracy of motor
		motor[motorA] = 10;
		while(nMotorEncoder[motorA] < 4) {}
		motor[motorA] = 0;
		// Rotates launcher to angle
		nMotorEncoder[motorA] = 0;
		wait1Msec(30);
		motor[motorA] = 15;
		while (nMotorEncoder[motorA] < (mRotations)) {}
	}
	else
	{
		// slightly adjusts to account for inaccuracy of motor
		motor[motorA] = -10;
		while(nMotorEncoder[motorA] > -4) {}
		motor[motorA] = 0;
		// Rotates launcher to angle
		nMotorEncoder[motorA] = 0;
		wait1Msec(30);
		motor[motorA] = -15;
		while (nMotorEncoder[motorA] > (mRotations)) {}
	}
	motor[motorA] = 0;
	return mRotations;
}

// Checks range and fires when range clear
void fire (float y, float z)
{
	float distance = z - 20;
	int msg = 0;
	bool rangeClear = false;
	float senseValue = 0;
	// Determine if area between the launcher and balloon is clear
	do
	{
		senseValue = SensorValue[S1];
		if (!rangeClear)
		{
			displayString(0, "Range not clear");
			wait1Msec(75);
		}
		if (senseValue > (distance)|| senseValue > 220)
		{
			// Checks if scout robot senses anything in the way
			sendMessage(2);
			wait1Msec(75);
			// Waits for range status from scout
			while (!bQueuedMsgAvailable())
			{
				wait1Msec(75);
			}
			while (bQueuedMsgAvailable())
			{
				msg = message;
				ClearMessage();
				wait1Msec(100);
			}
			ClearMessage();
			if (msg == 2)
				rangeClear = true;
			else
			{
				displayString(1, "Scout");
				displayString(3, "%i", msg);
			}
		}
		else
		{
			displayString(1, "Launcher");
			displayString(2, "%f", senseValue);
		}
		wait1Msec(250);
	} while (!rangeClear);

	eraseDisplay();
	// Sends message to scout indicating it is firing
	for (int i = 0; i < 5; i++)
	{
		sendMessage(3);
		wait1Msec(75);
	}
	displayString(0,"Firing");

	// Waits for scout robot to move then fires
	wait1Msec(3000);
	nMotorEncoder[motorB] = 0;
	motor[motorB] = -100;
	wait1Msec(1500);
	motor[motorB] = 0;
	wait1Msec(1000);
	// Sends messages to scout indicating it has fired
	for (int i = 0; i < 5; i++)
	{
		sendMessage(1);
		wait1Msec(75);
	}
}

task main()
{
	// Clears any previously queued bluetooth messages
	while (bQueuedMsgAvailable())
		ClearMessage();

	// Press middle button to start firing sequence
	displayString(0,"Press to start");
	while (nNxtButtonPressed != 3) {}
	while (nNxtButtonPressed == 3) {}
	eraseDisplay();

	time1[T1] = 0;

	// Sends start message to scout robot
	for (int i = 0; i < 5; i++)
	{
		sendMessage(1);
		wait1Msec(75);
	}

	SensorType[S2] = sensorTouch;

	resetVertAngle();

	// Waits to recieve coordinates from scout robot
	while (!bQueuedMsgAvailable())
		wait1Msec(75);

	wait1Msec(500);
	// Gets coordinates from scout robot
	float x = 0, y = 0, z = 0;

	while (bQueuedMsgAvailable())
	{
		// Horizontal distance from launcher
		x = messageParm[0];
		// Vertical distance from scout to balloon
		y = messageParm[1];
		// Distance between scout starting position and board
		z = messageParm[2];
		wait1Msec(250);
		ClearMessage();
	}
	// Adds difference between launcher and scout heights
	y += 4;
	// Adds difference between scout start location and launcher
	z -= 10;

	float xzdist = sqrt(x * x + z * z);

	// Angles the launcher horizontally and vertically
	angleArmVert(xzdist,y);
	wait1Msec(500);
	int hRotations = 0;
	hRotations = angleArmHorz(x,z);

	SensorType[S1] = sensorSONAR;

	// Checks range and fire
	fire(xzdist, y, z);

	// Clears any leftover or unused messages
	while (bQueuedMsgAvailable())
	{
		int k = message;
		wait1Msec(75);
		ClearMessage();
	}

	displayString(0, "Waiting for result");

	// Waits to receive result from scout robot
	while (!bQueuedMsgAvailable())
		wait1Msec(75);

	eraseDisplay();
	displayString(0,"Got result");

	// Gets and displays results
	int result = message;

	// Clears any leftover or unused messages
	while (bQueuedMsgAvailable())
	{
		int k = message;
		wait1Msec(75);
		ClearMessage();
	}

	float timeSec = time1[T1] / 1000.0;

	displayString(0, "Completed");
	if (result == 4)
	{
		displayString(1, "Success");
	}
	else if (result == 5)
	{
		displayString(1, "Fail");
	}
	else
	{
		displayString(1, "Error");
	}
	// Display time taken to pop balloon
	displayString(2, "Time: %f", timeSec);

	while (nNxtButtonPressed != 3) {}
	while (nNxtButtonPressed == 3) {}

	eraseDisplay();
	// Resets horizontal and vertical angles
	nMotorEncoder[motorA] = 0;
	if (hRotations < 0)
	{
		motor[motorA] = 15;
		while (nMotorEncoder[motorA] < -(hRotations)) {}
	}
	else
	{
		motor[motorA] = -15;
		while (nMotorEncoder[motorA] > -(hRotations)) {}
	}
	motor[motorA] = 0;

	resetVertAngle();
}
