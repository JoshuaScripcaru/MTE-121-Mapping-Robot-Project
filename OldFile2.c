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
const int MAP_RESOLUTION = 20;
const int angleOffset = 5;
bool isBoundary[25][25];

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
}


void rotateToAngle(int angle, int motorPower)
{
	displayString(2, "rotating");
	int sign = 1;
	if(angle - getGyroDegrees(GYRO_SENSOR))
		sign = (angle - getGyroDegrees(GYRO_SENSOR))/abs(angle - getGyroDegrees(GYRO_SENSOR));

	motor[LEFT_MOTOR] = -sign*motorPower;
	motor[RIGHT_MOTOR] = sign*motorPower;

	while(abs(angle - getGyroDegrees(GYRO_SENSOR)) > 5)
	{}

	setMotorPower(0);
	displayString(2, "finished rotating");
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
	displayString(1, "                    ");
	displayString(1, "minAngle = %d", minAngle);
}


int adjustAngle(int minDistance)
{
	float Kp = 0.25;
	int error = minDistance - DISTANCE_FROM_WALL;
	return(error*Kp);
}

void updatePosition(float* position,int minAngle, int minDistance)
{
	position[2] = getGyroDegrees(GYRO_SENSOR);
	position[0] += MOVE_INTERVAL*cos(position[2]* DEGREES_TO_RADIANS);
	position[1] -= MOVE_INTERVAL*sin(position[2]* DEGREES_TO_RADIANS);
	
	//displayString(3, "                 ");
	//displayString(3, "x: %.2f y: %.2f", position[0], position[1]);
}

void addBoundaryPoint(float*position)
{
	const int GRID_SIZE = 20;
	int row = position[0]/GRID_SIZE;
	int col = position[1]/GRID_SIZE;
	isBoundary[row][col] = true;
}


void putBoundaryIntoFile()
{
	TFileHandle fout;
	openWritePC(fout,"map.txt");
	//check for file open

	for(int row = 0; row < 25; row++)
	{
		string str = "";
		for(int col = 0; col < 25; col++)
		{
			if(isBoundary[row][col])
				writeTextPC(fout, "*");

			else
				writeTextPC(fout, "*");
		}
		writeEndlPC(fout);
	}

	closeFilePC(fout);

}

void mapToScreen()
{
	for (int row = 0; row < MAP_RESOLUTION; row++) 
	{
		string line = "";
		for (int col = 0; col < MAP_RESOLUTION; col++) 
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

	while(true) //shut down, when DISTANCE_FROM_WALL from starting position
	{
		int a = 0;
		int ds = 0;
		getMinAngle(a,ds); // Local scan
		int aaa = -a + adjustAngle(ds) + getGyroDegrees(GYRO_SENSOR); // Calculate angle to follow wall
		rotateToAngle(aaa,MOTOR_SPEED); // Rotate to the angle
		travelDistance(MOVE_INTERVAL,MOTOR_SPEED); // Drive some distance along the wall
		mapToScreen();
		addBoundaryPoint(position);
		updatePosition(position,a,ds);
	}

	motor[ULTRASONIC_MOTOR] = -MOTOR_SPEED;

	while(nMotorEncoder[ULTRASONIC_MOTOR] > 0)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	putBoundaryIntoFile();
}