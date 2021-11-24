#include "PC_FileIO.c"

//consts for motor
const tMotor LEFT_MOTOR = motorA;
const tMotor RIGHT_MOTOR = motorD;
const tMotor ULTRASONIC_MOTOR = motorB;

//consts for sensors
const tSensors ULTRASONIC_SENSOR = S1;
const tSensors GYRO_SENSOR = S4;
const tSensors COLOR_SENSOR = S2;

//consts for math
const float DEGREES_TO_RADIANS = PI/180;
const float WHEEL_RADIUS = 2.75;
const float ATAN_MAX_VALUE = PI/2;

//consts for mechanical components
const int GEAR_RATIO = 3;
const int MAX_MOTOR_SPEED = 100;

//consts for data collection
const int DISTANCE_FROM_WALL = 15;
const int ANGLE_OFFSET = 0;
const int ULTRASONIC_MOTOR_SPEED_SCAN = 200;
const int ULTRASONIC_MOTOR_SPEED_DEFAULT = 40;
const int LOOK_BACK_ANGLE = 90;

//consts for movement
const int MOVE_INTERVAL = 5;
const int MOTOR_SPEED = 20;
const int MIN_DISTANCE_TRAVEL = 50;

//consts for position tracking
const int X_VALUE_INDEX = 0;
const int Y_VALUE_INDEX = 1;
const int ANGLE_VALUE_INDEX = 2;

//const related to the map
const int MAP_RESOLUTION = 15; //how many cm resolution of the measuments taken
const int MAP_OFFSET = 4;
const int WRITE_INTERVAL = 5;
const int FILE_VALIDATION_LINE = 5;
const int DISTANCE_FROM_ORIGIN_TO_STOP = 10;
const int ARRAY_SIZE = 25; //number of row
bool isBoundary[ARRAY_SIZE][ARRAY_SIZE];

void setMotorPower(int motorPower) //sets left and right motor to specified power
{
	motor[RIGHT_MOTOR] = motor[LEFT_MOTOR] = motorPower;
}

void calibrateGyro() //calibrates the gyro
{
	const int DELAY_TIME = 100; //amount to delay gyro during calibration steps

	SensorType[GYRO_SENSOR] = sensorEV3_Gyro;
	wait1Msec(DELAY_TIME);
	SensorMode[GYRO_SENSOR] = modeEV3Gyro_Calibration;
	wait1Msec(DELAY_TIME);
	SensorMode[GYRO_SENSOR] = modeEV3Gyro_RateAndAngle;
	wait1Msec(DELAY_TIME);
}


void calibrateColorSensor() //calibrates the gyro
{
	const int DELAY_TIME = 100; //amount to delay gyro during calibration steps

	SensorType[COLOR_SENSOR] = sensorEV3_Color;
	wait1Msec(DELAY_TIME);
	SensorMode[COLOR_SENSOR] = modeEV3Color_Color;
	wait1Msec(DELAY_TIME);
}

void calibrateUltrasonic() //calibrates the ultrasonic
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

void travelDistance(int distance, int motorPower, int & prevMotorEncoder) //assumes the robot travels forward
{
	setMotorPower(motorPower);

	prevMotorEncoder = nMotorEncoder[RIGHT_MOTOR];
	float rotateBy = distance/(DEGREES_TO_RADIANS*WHEEL_RADIUS); //finds how to rotate the wheels
	float error = nMotorEncoder[RIGHT_MOTOR] - prevMotorEncoder - rotateBy; //find how many degrees the wheels need to rotate to

	while(error <= 0) //makes wheels rotate until error becomes negative
		error = nMotorEncoder[RIGHT_MOTOR] - prevMotorEncoder - rotateBy;

	setMotorPower(0);
}


void rotateToAngle(int angle, int motorPower)
{
	int sign = 1; //says if the robot needs to travel to an angle more positive or negative than what the gyro measures
	int error = angle - getGyroDegrees(GYRO_SENSOR);

	if(error) //sets 'sign' varible if it needs to turn a non zero amount
		sign = (error)/abs(error);

	motor[LEFT_MOTOR] = -sign*motorPower;
	motor[RIGHT_MOTOR] = sign*motorPower;

	while(sign*error >= 0) //sign and error are either both positive or negative. When error changes sign then the product is negative.
		error = angle - getGyroDegrees(GYRO_SENSOR);

	setMotorPower(0);
}

void getMinAngle(int & minAngle, int & minDistance)
{
	const int MAX_DISTANCE_POSSIBLE = 50;
	const float SCALE = 0.05;

	minDistance = MAX_DISTANCE_POSSIBLE;
	minAngle = 0;

	motor[ULTRASONIC_MOTOR] = -ULTRASONIC_MOTOR_SPEED_DEFAULT;

	while(nMotorEncoder[ULTRASONIC_MOTOR] > -180*GEAR_RATIO) //finds angle relative to robot of the closest point to the wall
	{
		int curDistance = SensorValue[ULTRASONIC_SENSOR];
		int curAngle = nMotorEncoder[ULTRASONIC_MOTOR];

		if(curDistance < minDistance)
		{
			minAngle = curAngle;
			minDistance = curDistance;
		}

		//adjusts motor speed such that is slows down when it is close to the wall
		int ultrasonicMotorSpeed = -ULTRASONIC_MOTOR_SPEED_SCAN/(ATAN_MAX_VALUE)*atan(curDistance*SCALE);

		if(abs(ultrasonicMotorSpeed) > MAX_MOTOR_SPEED)
			ultrasonicMotorSpeed = ultrasonicMotorSpeed*MAX_MOTOR_SPEED/abs(ultrasonicMotorSpeed);

		motor[ULTRASONIC_MOTOR] = ultrasonicMotorSpeed;
	}

	motor[ULTRASONIC_MOTOR] = ULTRASONIC_MOTOR_SPEED_DEFAULT; //rotates motor back to original position

	while(nMotorEncoder[ULTRASONIC_MOTOR] < LOOK_BACK_ANGLE*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	minAngle = -minAngle/GEAR_RATIO + ANGLE_OFFSET; //adjusts for the fact a gear is used
}


int adjustAngle(int minDistance) //adjusts angle to travel so robot stays specified distance from wall
{
	const float KP = 1.5;
	const int MAX_ADJUST_ANGLE = 30;

	int error = minDistance - DISTANCE_FROM_WALL; //finds how off the robot is from the desired distance
	float adjustBy = error*KP;

	if(fabs(adjustBy) > MAX_ADJUST_ANGLE) //makes the abs of the angle to adjust by no larger than MAX_ADJUST_ANGLE
		adjustBy = MAX_ADJUST_ANGLE*adjustBy/fabs(adjustBy);

	return(adjustBy);
}

void updatePosition(float* position, int prevMotorEncoder) //updates the position of the robot
{
	position[ANGLE_VALUE_INDEX] = getGyroDegrees(GYRO_SENSOR);

	position[X_VALUE_INDEX] += (nMotorEncoder[RIGHT_MOTOR]-prevMotorEncoder)*(DEGREES_TO_RADIANS*WHEEL_RADIUS)*cos(position[ANGLE_VALUE_INDEX]* DEGREES_TO_RADIANS);
	position[Y_VALUE_INDEX] -= (nMotorEncoder[RIGHT_MOTOR]-prevMotorEncoder)*(DEGREES_TO_RADIANS*WHEEL_RADIUS)*sin(position[ANGLE_VALUE_INDEX]* DEGREES_TO_RADIANS);
}

void addBoundaryPoint(float*position, int minAngle, int minDistance)
{
	int row = (position[X_VALUE_INDEX]+minDistance*sin((minAngle-position[ANGLE_VALUE_INDEX])* DEGREES_TO_RADIANS))/MAP_RESOLUTION + MAP_OFFSET;
	int col = (position[Y_VALUE_INDEX]-minDistance*cos((minAngle-position[ANGLE_VALUE_INDEX])* DEGREES_TO_RADIANS))/MAP_RESOLUTION + MAP_OFFSET;

	if(row < 0 || row >= ARRAY_SIZE)  //if row is not valid is doesn't add the value to the array
		return;

	if(col < 0 || col >= ARRAY_SIZE) //if column is not valid is doesn't add the value to the array
		return;

	isBoundary[row][col] = true;
}

float distanceFromOrigin(float*position)
{
	return(sqrt(pow(position[X_VALUE_INDEX],2)+pow(position[Y_VALUE_INDEX],2)));
}


bool putBoundaryIntoFile() //outputs to the file
{
	TFileHandle fout;
	bool didOpen = openWritePC(fout,"map.txt");

	if(!didOpen)
		return false;

	for(int row = 0; row < ARRAY_SIZE; row++)
	{
		for(int col = 0; col < ARRAY_SIZE; col++)
		{
			if(isBoundary[row][col])
				writeTextPC(fout, "*"); //puts a star if there is a boundary

			else
				writeTextPC(fout, " "); //puts a space if there is not a boundary
		}
		writeEndlPC(fout); //go to new line
	}

	closeFilePC(fout); //closes file

	return true;
}

task main()
{
	//calibrates sensors
	calibrateGyro();
	calibrateUltrasonic();
	calibrateColorSensor();
	nMotorEncoder[ULTRASONIC_MOTOR] = 0;

	motor[ULTRASONIC_MOTOR] = ULTRASONIC_MOTOR_SPEED_DEFAULT; //moves the ultrasonic back

	while(nMotorEncoder[ULTRASONIC_MOTOR] < LOOK_BACK_ANGLE*GEAR_RATIO)
	{}

	motor[ULTRASONIC_MOTOR] = 0;

	waitPressReleaseButton(); // Ask user to press button to start

	int count = 0;
	int prevMotorEncoder = 0; //used for position tracking
	float position[3] = {0,0,0}; //x position, y position, angle of robot


	//puts a low limit  for how much the robot can travel
	const int MIN_ILITERATIONS = MIN_DISTANCE_TRAVEL/MOVE_INTERVAL;

	while(count < MIN_ILITERATIONS || (distanceFromOrigin(position) > DISTANCE_FROM_ORIGIN_TO_STOP && SensorValue[COLOR_SENSOR] != (int)colorRed)) //shut down, when DISTANCE_FROM_WALL from starting position
	{
		int minAngle = 0;
		int minDistance = 0;

		getMinAngle(minAngle,minDistance); // Local scan

		int updatedAngle = - minAngle + getGyroDegrees(GYRO_SENSOR) + adjustAngle(minDistance); // Calculate angle to follow wall

		rotateToAngle(updatedAngle,MOTOR_SPEED); // Rotate to the angle
		travelDistance(MOVE_INTERVAL,MOTOR_SPEED, prevMotorEncoder); // Drive some distance along the wall

		addBoundaryPoint(position,minAngle,minDistance); //updates the boundary array
		updatePosition(position, prevMotorEncoder); //finds new position of the robot

		if(count%WRITE_INTERVAL == 0) //creates and updates file of map
		{
			bool fileOpen = putBoundaryIntoFile();
			if(!fileOpen)
			{
				displayString(FILE_VALIDATION_LINE, "File did not open");
			}
			else
			{
				displayString(FILE_VALIDATION_LINE, "File did     open");
			}
		}

		count++;
	}

	motor[ULTRASONIC_MOTOR] = -ULTRASONIC_MOTOR_SPEED_DEFAULT;  //rotate motor back to original position

	while(nMotorEncoder[ULTRASONIC_MOTOR] > 0)
	{}

	motor[ULTRASONIC_MOTOR] = 0; //stop motor

	putBoundaryIntoFile(); //creates and updates file of map
}
