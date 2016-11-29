package utilities;

import chassis.LCDInfo;

/**
 * Constants (mainly physical) used throughout the robot
 * @version 3.0
 * 
 */
public class Util {
	// field
	public static final double SQUARE_LENGTH = 30.48; 		//cm
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;
	public static final double GRIDLINE_THRESHOLD	= 0.1;	//Error margin (in cm) to consider position at a gridline
	public static final double ZONE_THRESHOLD		= 1.0;	//Error margin (in cm) for position at the green/red zone
	
	// blocks
	public static final double[] FOAM_RGB_VECTOR	= {0.4101, 0.5421, 0.7334}; //RGB normalized in lab lighting
	protected static final double COLOR_BW 			= 0.2;	// Bandwidth before a RGB value is no longuer the one expected
	public static final double VECTOR_TOLERANCE		= 0.95; //Minimum value of cross product between detected vector and FOAM_RGB_VECTOR
	public static final double FOAM_HEIGHT			= 4.9;	//Height of the styrofoam blocks
	public static final double FOAM_WIDTH			= 10;	//Height of the styrofoam blocks
	public static final double WOOD_MIN_WIDTH		= 20.0;	//minimum distance to travel to avoid wooden block
	
	// search
	public static final double SEARCH_DISTANCE 		= 55; 	//distance (in cm) to detect object
	public final static float BLOCK_DISTANCE		= 8.0f; //distance (in cm) to detect block type
	public static final double BACKUP_DISTANCE		= 4.0;
	public static final double NORTH_MAX 			= 10*SQUARE_LENGTH;	// max Y position (in cm) to search for items to avoid wall
	public static final double SOUTH_MAX			= -0.5*SQUARE_LENGTH;	// min Y position (in cm) to search for items to avoid wall
	public static final double EAST_MAX 			= 10*SQUARE_LENGTH;	// max X position (in cm) to search for items to avoid wall
	public static final double WEST_MAX 			= -0.5*SQUARE_LENGTH;	// min X position (in cm) to search for items to avoid walls
	public static final double SEARCH_FOV			= 60 * Math.PI / 180;
	public static final double APPROACH_BLOCK		= 2.0; //Extra distance to move toward styrofoam block to ensure that it isn't too far to reliably grab
	
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
	
	public static final int ROBOT_WIDTH				= 25;	//width (in cm) of collision rectangle around robot
	public static final int ROBOT_LENGTH			= 30;   //length (in cm) of collision rectangle around robot
	
	// capture
	public static final double FORKLIFT_HEIGHT		= 20.0;	//Length (in cm) of forklift 
	public static final double FORKLIFT_ROPE_RADIUS	= 1.45; //Radius (in cm) of circle that the rope wraps around to lift the forklift
	public static final int GRIP_STRENGTH			= 180; //Angle by which to rotate claw motor. In degrees because the lejos rotate functions are.
	public static final double TOWER_DISTANCE		= 8.0; //Distance (in cm) from tower before forklift is lowered
	public static final int FORKLIFT_ACCEL			= 300; //Acceleration of forklift (deg/s^2) 
	public static final int CLAW_ACCEL				= 500;
	public static final double CLAW_TO_CENTER		= 27.5;
	public static final double DROP_SPACE			= 1.0; //Distance above tower to drop next block from when stacking
	public static final double GRIP_THRESHOLD		= -150.5; //If tacho count of claw motor is greater than this, it is holding a block
	
	// odometry
	public static final boolean[] UPDATE_ALL = new boolean[] {true,true,true};
	public static final long ODOMETER_PERIOD		= 25; 	//Hz
	
	// navigation
	public static final int NAV_ACCELERATION		= 900;
	public static final double DEG_TOLERANCE		= 3.0; //degrees
	public static final double CM_TOLERANCE			= 1.0; //cm
	public static final double TRAVELTO_BW 			= 10;  //BW (in cm) at which it is safe to stop verifying for obstacles while traveling to a point
	
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
	public static final String IP_ADDR				= "192.168.2.3";
	public static final int TEAM_NUMBER				= 15;
	public static final boolean USE_WIFI			= true;
}
