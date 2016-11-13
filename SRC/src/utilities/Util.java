package utilities;

import chassis.LCDInfo;

/**
 * Constants (mainly physical) used throughout the robot
 * @version
 * @author juliette
 * 
 */
public class Util {
	// field
	public static final double SQUARE_LENGTH = 30.48; 		//cm
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;
	public static final double GRIDLINE_THRESHOLD	= 0.1;	//Error margin (in cm) to consider position at a gridline
	public static final double ZONE_THRESHOLD		= 1.0;	//Error margin (in cm) for position at the green/red zone
	
	// blocks
	public static final double FOAM_HEIGHT			= 4.9;	//Height of the styrofoam blocks
	public static final double FOAM_WIDTH			= 10;	//Height of the styrofoam blocks
	public static final double WOOD_MIN_WIDTH		= 15;	//minimum distance to travel to avoid wooden block
	
	// search
	public static final double SEARCH_DISTANCE 		= 45; 	//distance (in cm) to detect object
	public final static float BLOCK_DISTANCE		= 5.0f; //distance (in cm) to detect block type
	
	// avoider
	public static final double AVOID_DISTANCE		= 15; 	//distance (in cm) to avoid object	
	
	
	/**
	 *        \___ ___/
	 *            |
	 *         -------   ---
	 *         |+---+|    |
	 *         |+---+|  LENGTH
	 *         | EV3 |    |
	 *         |_____|   ---
	 *         
	 *         +WIDTH+
	 */
	
	public static final int ROBOT_WIDTH				= 20;	//width (in cm) of collision rectangle around robot
	public static final int ROBOT_LENGTH			= 30;   //length (in cm) of collision rectangle around robot
	
	// capture
	public static final double FORKLIFT_HEIGHT		= 24.0;	//Length (in cm) of forklift 
	public static final double FORKLIFT_ROPE_RADIUS	= 1.0; //Radius (in cm) of circle that the rope wraps around to lift the forklift
	public static final int GRIP_STRENGTH			= 180; //Angle by which to rotate claw motor. In degrees because the lejos rotate functions are.
	public static final double TOWER_DISTANCE		= 8.0; //Distance (in cm) from tower before forklift is lowered
	public static final int FORKLIFT_ACCEL			= 300; //Acceleration of forklift (deg/s^2) 
	
	// odometry
	public static final boolean[] UPDATE_ALL = new boolean[] {true,true,true};
	public static final long ODOMETER_PERIOD		= 25; 	//Hz
	
	// navigation
	public static final int NAV_ACCELERATION		= 4000;
	public static final double DEG_TOLERANCE		= 3.0; //degrees
	public static final double CM_TOLERANCE			= 1.0; //cm
	
	// motors
	public static final int MOTOR_FAST				= 200; //deg/sec
	public static final int MOTOR_SLOW				= 100; //deg/sec
	
	// robot
	public static final double WHEEL_RADIUS			= 2.18; //cm
	public static final double TRACK				= 16.04; //cm 
	public static final double US_TO_CENTER			= 12.0; //cm from us sensor to center of wheels
	public static final double INTENSITY_TO_CENTER	= 12; 	//cm from ligh sensor to center of wheels

	// sensors
	public static final int SLEEP_PERIOD			= 1000;  //ms to wait between each thread check;
	public static final int US_SAMPLES				= 6; 	//Default amount of samples to take for median filter
	public static final double SCAN_THETA_THRESHOLD = Math.PI/60;
		
	// lcd
	public static LCDInfo lcd;
	
	public static void setLCD(LCDInfo lcd) {
		Util.lcd = lcd;
	}
	
	//Wifi
	public static final String IP_ADDR				= "192.168.2.32";
	public static final int TEAM_NUMBER				= 15;
	public static final boolean USE_WIFI			= true;
}
