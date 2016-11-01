package utilities;

import chassis.LCDInfo;

public class Util {
	// field
	public static final double SQUARE_LENGTH = 30.48; //cm
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;
	
	// robot
	public static LCDInfo lcd;
	public static final long ODOMETER_PERIOD = 25; //Hz
	public static final double WHEEL_RADIUS = 2.025; //cm
	public static final double TRACK = 15.0; //cm 
	public static final double US_TO_CENTER = 4.50; //cm from us sensor to center of wheels
	public static final double INTENSITY_TO_CENTER = 12; //cm from ligh sensor to center of wheels

	// GREEN/RED coordinates
	public static final double [] GOAL_ZONE = {80, 80}; //TODO not actual value, to be defined
	public static void setLCD(LCDInfo lcd) {
		Util.lcd = lcd;
	}
}
