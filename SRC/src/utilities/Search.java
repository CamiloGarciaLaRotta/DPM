package utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import chassis.ColorSensor;
import chassis.Main;
import chassis.Main.RobotState;
import chassis.USSensor;
import utilities.Odometer.TURNDIR;

/**
 * 
 * @author juliette
 * @version 0.1
 * 
 * Blue styrofoam block search functionality for the robot
 */
public class Search extends Thread {
	
	// temporary hardcoded values, awaiting wifi module
	// coordinates: ((bottomLeft x,y), (upperRight x,y))
	private double[][] GREEN;
	private double[] N;
	private double[] S;
	private double[] E;
	private double[] W;
	private double[][] cardinals = new double[][] {N,W,S,E};
	private int currCardinal;
	
	private ArrayList<double[]> objectLocations = new ArrayList<double[]>();

	private static final int US_SAMPLES = 10;
	private Odometer odo;
	private USSensor usSensor;
	private ColorSensor colorSensor;
	private float lastDistanceDetected;
	private final float DISTANCE_THRESHOLD = 25; //cm
	private final static float BLOCK_DISTANCE = 4.0f; //distance to detect block type in cm
	public static double[] blockLocation;
	public static double[] obstacleLocation;
	private Navigation nav;
	
	public static final float[] STYROFOAM_COLOR = new float[] {0.0f,1.0f,1.0f};
	public enum SearchState {Default, AtCardinal, Scanning, Retrieving};
	public static SearchState searchState = SearchState.Default;
	
	
	/**
	 * Constructor for Search Class
	 * @param odometer - Odometer object
	 * @param colorSensor - ColorSensor object
	 * @param usSensor - USSensor object
	 * 
	 * @author Juliette Regimbal
	 * @version 
	 */
	public Search(Odometer odometer, ColorSensor colorSensor, USSensor usSensor) {
		this.odo = odometer;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		nav = new Navigation(odo);
		
		GREEN = new double[][]{{6*Util.SQUARE_LENGTH,3*Util.SQUARE_LENGTH},
								{8*Util.SQUARE_LENGTH,4*Util.SQUARE_LENGTH}}; 
		
		// mid points of the GREEN box
		double midX = (GREEN[0][0] + GREEN[1][0]) / 2;
		double midY = (GREEN[1][0] + GREEN[1][1]) / 2;
		
		// cardinal search points
		this.S = new double[] {midX, GREEN[0][1]};
		this.N = new double[] {midX, GREEN[1][1]};
		this.W = new double[] {GREEN[0][0], midY};
		this.E = new double[] {GREEN[1][0], midY};
		
		this.currCardinal = 0;
	}
	
	/**
	 * Starts search thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(Main.state != Main.RobotState.Search) {
			//check if interrupted
			if(Thread.interrupted()) return;
			//wait for localization to finish
			try {
				Thread.sleep(300);
			} catch (Exception e) {}
		}
		
		isStyrofoamBlock(); //Initialize rgb mode
		
		switch(searchState) {
		
		case Default: 
			// go to current cardinal point
			nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]); //TODO test if it breaks if block/RED
			
			// verify if navigation was interrupted
			if (Navigation.PathBlocked) {
				inspectObject();
			} else {
				searchState = SearchState.AtCardinal;
			}
			
			break;
			
		case AtCardinal:
			if(objectLocations.isEmpty()){ // no objects awaiting inspection, fill object list 
				// update only if its not the search being done
				if (currCardinal > 0) currCardinal++;
				
				// turn to start scanning angle
				double startAngle = 90 * currCardinal;
				nav.turnTo(startAngle, true);
				
				// start scatter search
				odo.spin(TURNDIR.CCW);
				while(odo.getTheta() < startAngle + 180) {
					// object found
					if (isObjectDetected()) {
						odo.stopMotors();
						
						// latch distance to object
						double distance = usSensor.getMedianSample(US_SAMPLES);
						objectLocations.add(new double[] {distance, odo.getTheta()});
						
						// resume scatter search
						odo.spin(TURNDIR.CCW);
					}
				}
				
				// sort list of object locations
				Collections.sort(objectLocations, new Comparator<double[]>() {
					@Override
					public int compare(double[] location1, double[] location2) {
						if (location1[0] == location2[0]) { 
				             return 0;
				        } else { 
				           return location1[0] > location2[0] ? 1 : -1;
				        }
					}
			    });
				
				searchState = SearchState.Scanning;
			} else { // objects awaiting inspection
				searchState = SearchState.Retrieving;
			}

			break;
			
		case Scanning: 
			// examine closest object
			double heading = objectLocations.get(0)[1];
			objectLocations.remove(0);
			nav.turnTo(heading, true);
			
			inspectObject();
			
			break;
			
		case Retrieving:
			// iddle state, waiting for capture to return and stack block
			try{
				Thread.sleep(500);
			} catch(Exception e) {}
			break;
		}
	}
	
	// when object is in sight, approach slowly and insect
	private void inspectObject() {
		odo.setMotorSpeed(70); // verify that the forward speed is adequate
		odo.forwardMotors();   // and create constant for its value
		
		while(usSensor.getMedianSample(US_SAMPLES) > 6); // make constant for this value
		odo.stopMotors();
		
		odo.setMotorSpeeds(60, 60); // make constant for this value
		odo.forwardMotors();
		
		while(usSensor.getMedianSample(US_SAMPLES) > BLOCK_DISTANCE); //wait until close enough to determine if it's a styrofoam block
		odo.stopMotors();// inspect object
		if(isStyrofoamBlock()) { 
			searchState = SearchState.Retrieving;
			Main.state = Main.RobotState.Capture;
		} else {
			// object was wooden block
			// return to cardinal point and inspect next object
			searchState = SearchState.Default;
		}	
	}

	/**
	 * 
	 * @return if the detected object is a styrofoam block
	 */
	private boolean isStyrofoamBlock() {
		return (colorSensor.getColor()[0] < colorSensor.getColor()[1]);
	}
	
	/**
	 * 
	 * @return if there is an object that can be detected by the robot
	 */
	private boolean isObjectDetected() {
		float currentDistance = usSensor.getMedianSample(US_SAMPLES);
		boolean isObject = (lastDistanceDetected - currentDistance > DISTANCE_THRESHOLD) && (currentDistance <= 45);
		lastDistanceDetected = currentDistance;
		return isObject;
	}
	
	/**
	 * Measures distance between two RGB color vectors
	 * @param a first RGB vector
	 * @param b second RGB vector
	 * @return magnitude of the distance between a and b
	 */
	public static float colorDistance(float[] a, float[] b){
		return (float)Math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
	}
}
