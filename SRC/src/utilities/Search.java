package utilities;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import chassis.ColorSensor;
import chassis.Main;
import chassis.Main.RobotState;
import chassis.USSensor;
import utilities.Avoider.AvoidState;
import utilities.Capture.CaptureState;
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
	private Rectangle GREEN;
	private Rectangle currRect;
	private double[][] GREEN_ARR;
	private double[] N;
	private double[] S;
	private double[] E;
	private double[] W;
	private double[][] cardinals = new double[][] {N,W,S,E};
	private int currCardinal;
	
	private double[] currPos = new double[3];
	
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
	public enum SearchState {Default, AtCardinal, AtDropZone, Inspecting, Iddle};
	public static SearchState searchState = SearchState.Default;
	
	//TODO TODO TODO TODO
	// - when latching angle for detected object, make sure it doesnt latch it multiple times -> give a min interval

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
		int green_width = 1;	// to be updated via constructor 
		int green_height = 2;	// depending on wifi RED coordinates
		this.GREEN = new Rectangle(6*30,3*30,green_width*40, green_height*40);

		this.GREEN_ARR = new double[][]{{6*Util.SQUARE_LENGTH,3*Util.SQUARE_LENGTH},
								{8*Util.SQUARE_LENGTH,4*Util.SQUARE_LENGTH}}; 
								
		this.currRect = new Rectangle();
		
		// mid points of the GREEN box
		double midX = (GREEN_ARR[0][0] + GREEN_ARR[1][0]) / 2;
		double midY = (GREEN_ARR[1][0] + GREEN_ARR[1][1]) / 2;
		
		// cardinal search points
		this.S = new double[] {midX, GREEN_ARR[0][1]};
		this.N = new double[] {midX, GREEN_ARR[1][1]};
		this.W = new double[] {GREEN_ARR[0][0], midY};
		this.E = new double[] {GREEN_ARR[1][0], midY};
		
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
		
		// build current position rectangle
		odo.getPosition(this.currPos);
		currRect.setBounds((int)(currPos[0]*Util.SQUARE_LENGTH), (int)(currPos[1]*Util.SQUARE_LENGTH),5, 5);
		
		switch(searchState) {
		
		case Default: 
			Avoider.avoidState = AvoidState.Disabled;
			
			if(currCardinal == -1) { // at origin, not yet at GREEN zone
				currCardinal++;
				nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
				
				// verify if navigation was interrupted
				if (Navigation.PathBlocked) {
					// does the robot already have a block?
					if(Capture.captureState == CaptureState.Disabled) inspectObject();
					else {
						Avoider.avoidState = AvoidState.Enabled;
						// wait for avoider to finish
						while(Main.state == RobotState.Avoiding);
					}
					
				}
			} else { 
				// no more object at that cardinal point
				if(objectLocations.isEmpty()) {
					currCardinal++;
					
					// linear set of instructions to reach next cardinal
					nav.turnBy(-90);
					odo.setMotorSpeeds(Util.SLOW_MOTOR, Util.SLOW_MOTOR);
					odo.forwardMotors();
					
					// advance until at axis of next cardinal
					String axis = (currCardinal % 2 == 0) ? "Y" : "X";
					switch(axis){
					case "X": 
						while(odo.getX() < cardinals[currCardinal][0] - 2 || 
								odo.getX() > cardinals[currCardinal][0] + 2);
						break;
					case "Y": 
						while(odo.getY() < cardinals[currCardinal][1] - 2 ||
								odo.getY() > cardinals[currCardinal][1] + 2);
						break;
					}
					odo.stopMotors();
				
					nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
					
					Search.searchState = SearchState.AtCardinal;
				} else {
					Search.searchState = SearchState.Inspecting;
				}				
			}
			
			Avoider.avoidState = AvoidState.Disabled;

			break;
			
		case AtCardinal:
			
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
			
			searchState = SearchState.Inspecting;

			break;
		
		case AtDropZone:
			nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
			searchState = SearchState.Default;
			break;
			
		case Inspecting: 
			
			// examine closest object
			double heading = objectLocations.get(0)[1];
			objectLocations.remove(0);
			nav.turnTo(heading, true);
			
			inspectObject();
			
			break;
			
		case Iddle:
		
			// iddle state, waiting for capture to return and stack block
			try{
				Thread.sleep(Util.SLEEP_PERIOD);
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
			searchState = SearchState.Iddle;
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
