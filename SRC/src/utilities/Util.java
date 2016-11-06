package utilities;

import chassis.LCDInfo;

public class Util {
	// field
	public static final double SQUARE_LENGTH = 30.48; //cm
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH; 
	
	// robot
	public static LCDInfo lcd;
	public static final long ODOMETER_PERIOD		= 25; 	//Hz
	public static final double WHEEL_RADIUS			= 2.18; //cm
	public static final double TRACK				= 16.6; //cm 
	public static final double US_TO_CENTER			= 4.50; //cm from us sensor to center of wheels
	public static final double INTENSITY_TO_CENTER	= 12; 	//cm from ligh sensor to center of wheels
	public static final double GRIDLINE_THRESHOLD	= 0.1;	//Error margin (in cm) to consider position at a gridline
	public static final double ZONE_THRESHOLD		= 1.0;	//Error margin (in cm) for position at the green/red zone
	public static final double BLOCK_HEIGHT			= 4.9;	//Height of the styrofoam blocks
	public static final int US_SAMPLES				= 6; 	//Default amount of samples to take for median filter
	public static final double FORKLIFT_HEIGHT		= 15.0;	//Length of forklift 

	public static final boolean[] UPDATE_ALL = new boolean[] {true,true,true};

	// special coordinates
	public static final double [] GOAL_ZONE = {80, 80}; //TODO not actual value, to be defined
	public static void setLCD(LCDInfo lcd) {
		Util.lcd = lcd;
	}
}
