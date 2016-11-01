package utilities;

import chassis.ColorSensor;
import chassis.Main;
import chassis.USSensor;
import lejos.hardware.Sound;
import utilities.Odometer.LINEDIR;

public class Search extends Thread {
	private static final int US_SAMPLES = 10;
	private Odometer odo;
	private USSensor usSensor;
	private ColorSensor colorSensor;
	private final float FIELD_BOUNDS = 65; //cm
	private float lastDistanceDetected;
	private final float DISTANCE_THRESHOLD = 25; //cm
	private final double fieldToSearch = Math.PI/2;
	private final float[][] corners = new float[][] {{0,0},{0,60.96f},{60.96f,60.96f},{60.96f,0.0f}};
	private int corner;
	private int dir;
	private final static float BLOCK_DISTANCE = 4.0f; //distance to detect block type in cm
	public static double[] blockLocation;
	public static double[] obstacleLocation;
	private final int APPROACH_SPIN_SPEED = 75;
	private Navigation nav;
		
	public static final float[] STYROFOAM_COLOR = new float[] {0.0f,1.0f,1.0f};
	
	public Search(Odometer odo, ColorSensor colorSensor, USSensor usSensor) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.corner = 0;
		this.dir = 1;
		nav = new Navigation(odo);
	}
	
	@Override
	public void run() {
		while(Main.state != Main.RobotState.Search) {
			//wait for localization to finish
			try {
				Thread.sleep(300);
			} catch (Exception e) {}
		}
		
		boolean found = false;
		
		// search algorithm here
			
		//Styrofoam block found - begin capture
		Main.state = Main.RobotState.Capture; 
	}
	
	private boolean isStyrofoamBlock() {
		return (colorSensor.getColor()[0] < colorSensor.getColor()[1]);
	}
	
	private boolean isObjectDetected() {
		float currentDistance = usSensor.getMedianSample(US_SAMPLES);
		boolean isObject = (lastDistanceDetected - currentDistance > DISTANCE_THRESHOLD) && (currentDistance <= 45);
		lastDistanceDetected = currentDistance;
		return isObject;
	}
	
	public static float colorDistance(float[] a, float[] b){
		return (float)Math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
	}
}
