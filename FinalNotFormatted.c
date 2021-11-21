#include "PC_FileIO.c"

const tMotor LEFT_MOTOR = motorA;
const tMotor RIGHT_MOTOR = motorD;
const tMotor ULTRASONIC_MOTOR = motorB;
const tSensors ULTRASONIC_SENSOR = S1;
const tSensors GYRO_SENSOR = S4;
const int GEAR_RATIO = 3;
const int DISTANCE_FROM_WALL = 15;
const int MOVE_INTERVAL = 5;
const int LOOK_BACK_ANGLE = 90;
const float DEGREES_TO_RADIANS = PI/180;
const int  MOTOR_SPEED = 20;
const int MAP_RESOLUTION = 15;
const int MAP_OFFSET = 4;
const int angleOffset = 0;
const int ARRAY_SIZE = 25;
bool isBoundary[ARRAY_SIZE][ARRAY_SIZE];

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

void waitPressReleaseButton()
{
	while(!getButtonPress(buttonEnter))
	{}

	while(getButtonPress(buttonAny))
	{}
}

void travelDistance(int distance, int motorPower, int & prevMotorEncoder)//, int* position) //needs int[3] &
{
	setMotorPower(motorPower);

	int initialCount = nMotorEncoder[RIGHT_MOTOR];
	prevMotorEncoder = nMotorEncoder[RIGHT_MOTOR];
	float rotateBy = distance*(180/(PI*2.75));
	float error = nMotorEncoder[RIGHT_MOTOR] - initialCount - rotateBy;

	while(error <= 0)
	{
		error = nMotorEncoder[RIGHT_MOTOR] - initialCount - rotateBy;
	}

	setMotorPower(0);
}


void rotateToAngle(int angle, int motorPower)
{
	//displayString(2, "rotating");
	int sign = 1;
	if(angle - getGyroDegrees(GYRO_SENSOR))
		sign = (angle - getGyroDegrees(GYRO_SENSOR))/abs(angle - getGyroDegrees(GYRO_SENSOR));

	motor[LEFT_MOTOR] = -sign*motorPower;
	motor[RIGHT_MOTOR] = sign*motorPower;

	while(sign*(angle - getGyroDegrees(GYRO_SENSOR)) >= 0)
	{}

	setMotorPower(0);
	//displayString(2, "finished rotating");
}

void getMinAngle(int & minAngle, int & minDistance)
{

	minDistance = 1000000;
	minAngle = 0;

	motor[ULTRASONIC_MOTOR] = -MOTOR_SPEED;

	while(nMotorEncoder[ULTRASONIC_MOTOR] > -180*GEAR_RATIO)
	{
		const float scale = 0.05;

		int curDistance = SensorValue[ULTRASONIC_SENSOR];

		motor[ULTRASONIC_MOTOR] = -2*MOTOR_SPEED/(2*PI*scale)*atan(curDistance*scale);

		if(curDistance < minDistance)
		{
			minAngle = nMotorEncoder[ULTRASONIC_MOTOR];
			minDistance = curDistance;
		}
	}

	motor[ULTRASONIC_MOTOR] = 3*MOTOR_SPEED;

	while(nMotorEncoder[ULTRASONIC_MOTOR] < LOOK_BACK_ANGLE*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	minAngle = -minAngle/GEAR_RATIO + angleOffset;
	//displayString(1, "                    ");
	//displayString(1, "minAngle = %d", minAngle);
}


int adjustAngle(int minDistance)
{
	float Kp = 1.5;
	int error = minDistance - DISTANCE_FROM_WALL;
	float adjustBy = error*Kp;

	if(fabs(adjustBy) > 30)
		adjustBy = 30*adjustBy/fabs(adjustBy);

	return(adjustBy);
}

void updatePosition(float* position, int prevMotorEncoder)
{
	position[2] = getGyroDegrees(GYRO_SENSOR);

	position[0] += MOVE_INTERVAL*cos(position[2]* DEGREES_TO_RADIANS);
	position[1] -= MOVE_INTERVAL*sin(position[2]* DEGREES_TO_RADIANS);

	displayString(3, "                 ");
	displayString(3, "x: %.2f y: %.2f", position[0], position[1]);
}

void addBoundaryPoint(float*position, int minAngle, int minDistance)
{
	int row = (position[0]+minDistance*sin((minAngle-position[2])* DEGREES_TO_RADIANS))/MAP_RESOLUTION + MAP_OFFSET;
	int col = (position[1]-minDistance*cos((minAngle-position[2])* DEGREES_TO_RADIANS))/MAP_RESOLUTION + MAP_OFFSET;

	if(row < 0 || row >= ARRAY_SIZE)
		return;

	if(col < 0 || col >= ARRAY_SIZE)
		return;

	isBoundary[row][col] = true;
}

float distanceFromOrigin(float*position)
{
	return(sqrt(pow(position[0],2)+pow(position[1],2)));
}


void putBoundaryIntoFile()
{
	TFileHandle fout;
	openWritePC(fout,"map.txt");
	//check for file open

	for(int row = 0; row < ARRAY_SIZE; row++)
	{
		string str = "";
		for(int col = 0; col < ARRAY_SIZE; col++)
		{
			if(isBoundary[row][col])
				writeTextPC(fout, "*");

			else
				writeTextPC(fout, " ");
		}
		writeEndlPC(fout);
	}

	closeFilePC(fout);

}

void mapToScreen()
{
	for (int row = 0; row < ARRAY_SIZE; row++)
	{
		string line = "";
		for (int col = 0; col < ARRAY_SIZE; col++)
		{
			if (isBoundary[row][col])
				strcat(line, "*");

			else
				strcat(line, " ");
		}
		displayString(row, line);
	}
}



task main()
{
	calibrateGyro();
	calibrateUltrasonic();
	nMotorEncoder[ULTRASONIC_MOTOR] = 0;

	motor[ULTRASONIC_MOTOR] = MOTOR_SPEED;

	while(nMotorEncoder[ULTRASONIC_MOTOR] < LOOK_BACK_ANGLE*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	float position[3] = {0,0,0};

	waitPressReleaseButton(); // Ask user to press button to start

	int count = 0;
	int prevMotorEncoder = 0;

	/*
	int a = 0;
	int ds = 0;
	getMinAngle(a,ds); // Local scan
	int aaa = adjustAngle(ds) + getGyroDegrees(GYRO_SENSOR); // Calculate angle to follow wall
	displayString(3, "                 ");
	displayString(3, "%d", adjustAngle(ds));
	rotateToAngle(aaa,MOTOR_SPEED); // Rotate to the angle
	wait1Msec(10000);
	*/

	/*
	int n = 12;

	for(int i = 0; i < n; i++)
	{

		for(int j = 0; j < 1; j++)
		{
			travelDistance(MOVE_INTERVAL,MOTOR_SPEED,prevMotorEncoder);
			wait1Msec(50);
			updatePosition(position,prevMotorEncoder);
		}



		rotateToAngle(360/n *(i+1),MOTOR_SPEED);
		displayString(2, "             ",);
		displayString(2, "Deg: %d", getGyroDegrees(GYRO_SENSOR));
	}

	wait1Msec(10000);
	*/





	while((count < 10 || distanceFromOrigin(position) > 10 ) && count < 100) //shut down, when DISTANCE_FROM_WALL from starting position
	{
		int a = 0;
		int ds = 0;
		getMinAngle(a,ds); // Local scan
		int aaa = -a + getGyroDegrees(GYRO_SENSOR) + adjustAngle(ds); // Calculate angle to follow wall
		rotateToAngle(aaa,MOTOR_SPEED); // Rotate to the angle
		travelDistance(MOVE_INTERVAL,MOTOR_SPEED, prevMotorEncoder); // Drive some distance along the wall
		mapToScreen();
		addBoundaryPoint(position,a,ds);
		updatePosition(position, prevMotorEncoder);
		putBoundaryIntoFile();
		count++;
	}

	if(count >= 100)
		displayString(4, "Stopped due to counter", position[0], position[1]);


	motor[ULTRASONIC_MOTOR] = -MOTOR_SPEED;

	while(nMotorEncoder[ULTRASONIC_MOTOR] > 0)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	putBoundaryIntoFile();

	wait1Msec(100000);
}
