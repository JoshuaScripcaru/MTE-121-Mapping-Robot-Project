const tMotor LEFT_MOTOR = motorD;
const tMotor RIGHT_MOTOR = motorA;
const tMotor ULTRASONIC_MOTOR = motorB;
const tSensors ULTRASONIC_SENSOR = S1;
const tSensors GYRO_SENSOR = S4;
const int GEAR_RATIO = 3;
const int DISTANCE_FROM_WALL = 20;
const int MOVE_INTERVAL = 5;

void setMotorPower(int motorPower)
{
	motor[RIGHT_MOTOR] = motor[LEFT_MOTOR] = motorPower;
}

void calibrateGyro()
{
	SensorType[GYRO_SENSOR] = sensorEV3_Gyro;
	wait1Msec(100);
	SensorMode[GYRO_SENSOR] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[GYRO_SENSOR] = modeEV3Gyro_RateAndAngle;
}

void calibrateUltrasonic()
{
	SensorType[ULTRASONIC_SENSOR] = sensorEV3_Ultrasonic;
}

/*
void rotateMotor(int angle, int motorPower)
{
	setMotorPower(motorPower);

	int initialCount = nMotorEncoder[RIGHT_MOTOR];
	int error = nMotorEncoder[RIGHT_MOTOR] - initialCount - angle;

	while(abs(error) > 5)
	{
		error = nMotorEncoder[RIGHT_MOTOR] - initialCount - angle;
	}

	setMotorPower(0);
}
*/

void waitPressReleaseButton()
{
	while(!getButtonPress(buttonEnter))
	{}

	while(getButtonPress(buttonAny))
	{}
}

void travelDistance(int distance, int motorPower)//, int* position) //needs int[3] &
{
	setMotorPower(motorPower);

	int initialCount = nMotorEncoder[RIGHT_MOTOR];
	float rotateBy = distance*(180/(PI*2.75));
	float error = nMotorEncoder[RIGHT_MOTOR] - initialCount - rotateBy;

	while(fabs(error) > 5)
	{
		error = nMotorEncoder[RIGHT_MOTOR] - initialCount - rotateBy;
	}

	setMotorPower(0);

	// Update position array
// Account for small angle change during movement by taking
// average of the angle before and after the movement
	/*
	float angle = (getGyroDegrees(GYRO_SENSOR) + position[2]) / 2;
	position[2] = getGyroDegrees(GYRO_SENSOR);
	position[0] += distance*cos(angle);
	position[1] += distance*sin(angle);
	*/
}

/*
void rotateByAngle(int angle, int motorPower)
{
	motor[LEFT_MOTOR] = motorPower;
	motor[RIGHT_MOTOR] = -motorPower;

	int initialAngle = getGyroDegrees(ULTRASONIC_SENSOR);
	int error = getGyroDegrees(ULTRASONIC_SENSOR) - initialAngle - angle;

	while(abs(error) > 5)
	{
		error = getGyroDegrees(ULTRASONIC_SENSOR) - initialAngle - angle;
	}

	setMotorPower(0);
}
*/


// Rotating
void rotateToAngle(int angle, int motorPower)
{
	int sign = 1;
	if(angle - getGyroDegrees(GYRO_SENSOR))
		sign = (angle - getGyroDegrees(GYRO_SENSOR))/abs(angle - getGyroDegrees(GYRO_SENSOR));

	motor[LEFT_MOTOR] = -sign*motorPower;
	motor[RIGHT_MOTOR] = sign*motorPower;

	while(abs(angle - getGyroDegrees(GYRO_SENSOR)) > 5)
	{}

	setMotorPower(0);
}

void getMinAngle(int & minAngle, int & minDistance)
	{

		minDistance = 1000000;
		minAngle = 0;

		motor[ULTRASONIC_MOTOR] = -20;

		while(nMotorEncoder[ULTRASONIC_MOTOR] > -180*GEAR_RATIO)
	{
		int curDistance = SensorValue[ULTRASONIC_SENSOR];
		if(curDistance < minDistance)
		{
			minAngle = nMotorEncoder[ULTRASONIC_MOTOR];
			minDistance = curDistance;
		}
	}

	motor[ULTRASONIC_MOTOR] = 20;

	while(nMotorEncoder[ULTRASONIC_MOTOR] < 30*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	minAngle = -minAngle/GEAR_RATIO;
}

/*
void adjustToAngleAndTravel(int* position)
{

	int[2] values;
	getMinAngle(values);
	rotateToAngle(angleToAdjust + 90, 20); //may be -90 instead
	travelDistance(5,20, position);

}
*/


int adjustAngle(int minDistance)
{
	float Kp = 1.75;
	int error = minDistance - DISTANCE_FROM_WALL;
	return(error*Kp);
}


// TODO function to add local map to global map
// TODO function to output global map to screen

task main()
{
	calibrateGyro();
	calibrateUltrasonic();
	nMotorEncoder[ULTRASONIC_MOTOR] = 0;

	motor[ULTRASONIC_MOTOR] = 20;

	while(nMotorEncoder[ULTRASONIC_MOTOR] < 30*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;


	// Set up variables and arrays
	// Create robot position 2D array
	/*
	float robotPosition[3]; // x, y, angle
	bool plzWork[25][25];
	int pos[3];
	*/

	//bool globalMap[1][1];
	// Create local map 2D array
	//bool localMap[160];

	// Ask user to press button to start
	// Define which side the wall is on thru button press
	// While the perimeter map is not closed
	// Local scan
	// Add local scan to global scan
	// Calculate angle to follow wall
	// Rotate to the angle
	// Drive some distance along the wall

	waitPressReleaseButton();

	while(true)
	{
		int a = 0;
		int ds = 0;
		getMinAngle(a,ds);
		//int aa = -a + getGyroDegrees(GYRO_SENSOR);
		//rotateToAngle(aa,20);
		int aaa = -a + adjustAngle(ds) + getGyroDegrees(GYRO_SENSOR);
		rotateToAngle(aaa,20);
		travelDistance(MOVE_INTERVAL,20);
	}

	/*

	while(true)
	{
		adjustToAngleAndTravel(pos);
	}
	*/

	motor[ULTRASONIC_MOTOR] = -20;

	while(nMotorEncoder[ULTRASONIC_MOTOR] > 0)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

// End of while loop
// Output final map to screen or file


}
