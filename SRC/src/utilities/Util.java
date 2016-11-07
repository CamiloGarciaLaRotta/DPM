package utilities;

import chassis.LCDInfo;

public class Util {
	// field
	public static final double SQUARE_LENGTH = 30.48; 		//cm
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;
	public static final double GRIDLINE_THRESHOLD	= 0.1;	//Error margin (in cm) to consider position at a gridline
	public static final double ZONE_THRESHOLD		= 1.0;	//Error margin (in cm) for position at the green/red zone
	
	// blocks
	public static final double FOAM_HEIGHT			= 4.9;	//Height of the styrofoam blocks
	public static final double WOOD_MIN_WIDTH		= 15;	//minimum distance to travel to avoid wooden block
	
	// search
	public static final double SEARCH_DISTANCE 		= 25; 	//distance (in cm) to detect object 
	
	// avoider
	public static final double AVOID_DISTANCE		= 15; 	//distance (in cm) to avoid object	
	public static final int ROBOT_RECTANGLE			= 5;	//side (in cm) of square around rectangle for avoidance
	
	// capture
	public static final double FORKLIFT_HEIGHT		= 15.0;	//Length (in cm) of forklift 
	
	// odometry
	public static final boolean[] UPDATE_ALL = new boolean[] {true,true,true};
	public static final long ODOMETER_PERIOD		= 25; 	//Hz
	
	// motors
	public static final int SLOW_MOTOR 				= 60;   //deg/sec
	public static final int FAST_MOTOR 				= 150;  //deg/sec	
	
	// robot
	public static final double WHEEL_RADIUS			= 2.18; //cm
	public static final double TRACK				= 16.4; //cm 
	public static final double US_TO_CENTER			= 12.0; //cm from us sensor to center of wheels
	public static final double INTENSITY_TO_CENTER	= 12; 	//cm from ligh sensor to center of wheels

	// sensors
	public static final int SLEEP_PERIOD			= 1000;  //ms to wait between each thread check;
	public static final int US_SAMPLES				= 6; 	//Default amount of samples to take for median filter
		
	// lcd
	public static LCDInfo lcd;
	
	public static void setLCD(LCDInfo lcd) {
		Util.lcd = lcd;
	}
}
