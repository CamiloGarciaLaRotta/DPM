package utilities;

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
 * Blue styrofoam block search functionality for the robot
 * @author juliette
 * @version 0.2
 * 
 *
 */
public class Search extends Thread {
	
	// Instances
	private Odometer odo;	
	private Navigation nav;
	private USSensor usSensor;
	private ColorSensor colorSensor;
	
	// Coordinates
	private double[] N = new double[2];
	private double[] W = new double[2];
	private double[] S = new double[2];
	private double[] E = new double[2];
	private double[][] cardinals = new double[4][2];
	private int currCardinal;
	
	// object detection
	private ArrayList<double[]> objectLocations = new ArrayList<double[]>();
	private float lastDistanceDetected;
	private double lastX, lastY;
	
	// states
	public enum SearchState {Default, AtCardinal, AtDropZone, Inspecting, Iddle};
	public static SearchState searchState;
	
	
	/**
	 * Constructor for Search Class
	 * @param odometer - Odometer object
	 * @param colorSensor - ColorSensor object
	 * @param usSensor - USSensor object
	 * @param GREEN - coordinates of the green scoring zone
	 * 
	 * @author Juliette Regimbal
	 * @version 
	 */
	public Search(Odometer odometer, ColorSensor colorSensor, USSensor usSensor, double[][] GREEN) {
		this.odo = odometer;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		nav = new Navigation(odo);
		
		Search.searchState = SearchState.Default;
		
		// mid points of the GREEN box
		double midX = (GREEN[0][0] + GREEN[1][0]) / 2;
		double midY = (GREEN[0][1] + GREEN[1][1]) / 2;
		
		// cardinal search points
		this.S = new double[] {midX, GREEN[0][1]};
		this.N = new double[] {midX, GREEN[1][1]};
		this.W = new double[] {GREEN[0][0], midY};
		this.E = new double[] {GREEN[1][0], midY};
		
		this.cardinals = new double[][] {N,W,S,E};
		
		this.lastDistanceDetected = -1;
		this.currCardinal = -1;
	}
	
	/**
	 * Starts search thread
	 * {@inheritDoc}
	 */
	@SuppressWarnings("deprecation")
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
		
		while(true){
			switch(searchState) {
			
			case Default: 
				// at origin, not yet at GREEN zone
				if(currCardinal == -1) { 
					
					// next cardinal, wrap around
					currCardinal++; currCardinal %= 4;
					
					nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
					// until it arrives at current cardinal
//					while(Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, 
//									new double[] {cardinals[currCardinal][0], cardinals[currCardinal][1]}) > 2){
//						
//						// travel to cardinal
//						if(odo.getMotors()[0].getRotationSpeed() +/- 5) {
//							nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
//						}
//						
//						// verify if navigation was interrupted
//						if (Navigation.PathBlocked) {
//							// does the robot already have a block?
//							if(Capture.captureState == CaptureState.Disabled) inspectObject();
//							else {
//								Avoider.avoidState = AvoidState.Enabled;
//								// wait for avoider to finish
//								while(Main.state == RobotState.Avoiding);
//							}
//						}	
//					}
					
					Search.searchState = SearchState.AtCardinal;
					
				} else { 
					
					// no more object at that cardinal point
					if(objectLocations.isEmpty()) {
						// next cardinal, wrap around
						currCardinal++; currCardinal %= 4;
						
						// linear set of instructions to reach next cardinal
						// at this step the robot is ensured to be on the old cardinal point
						
						// place on correct heading
						nav.turnTo((Math.PI + (currCardinal-1)*Math.PI/2), true);
						
						// travel until it reaches the axis of next cardinal point
						travelToAxis(true);
					
						// travel to the cardinal point without fear of bumping onto the tower
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
				double startAngle = Math.PI/2 * currCardinal;
				nav.turnTo(startAngle, true);
				
				// reinitialize last distance 
				this.lastDistanceDetected = -1;
				
				// start scatter search
				odo.setMotorSpeed(Util.MOTOR_SLOW);
				odo.spin(TURNDIR.CCW);
				double targetAngle = startAngle + Math.PI;
				while(Math.abs(Navigation.minimalAngle(odo.getTheta(), targetAngle)) > Util.SCAN_THETA_THRESHOLD) {
					// object found
					if (isObjectDetected()) {
						odo.stopMotors();
						
						// latch object's coordinates
						double distance = usSensor.getMedianSample(Util.US_SAMPLES);
						double heading = odo.getTheta();
						double X = odo.getX() + distance*Math.cos(heading);
						double Y = odo.getY() + distance*Math.sin(heading);
						objectLocations.add(new double[] {X,Y,distance,heading});
						
						// resume scatter search
						odo.setMotorSpeed(Util.MOTOR_SLOW);
						odo.spin(TURNDIR.CCW);
					}
				}
				
				odo.stopMotors();
				
				// sort list of object locations
				Collections.sort(objectLocations, new Comparator<double[]>() {
					@Override
					public int compare(double[] location1, double[] location2) {
						if (location1[2] == location2[2]) { 
				             return 0;
				        } else { 
				           return location1[2] > location2[2] ? 1 : -1;
				        }
					}
			    });
				
				searchState = SearchState.Inspecting;

				break;
			
			case AtDropZone:
				
				// back off until  to avoid colliding with tower
				travelToAxis(false);
				
				searchState = SearchState.Default;
				
				break;
				
			case Inspecting: 
				
				if(!objectLocations.isEmpty()) {
					// examine closest object
					
					double X = objectLocations.get(0)[0];
					double Y = objectLocations.get(0)[1];
					
					// turn towards object
					nav.turnTo(objectLocations.get(0)[3], true); 
					
					objectLocations.remove(0);
					
					inspectObject(X, Y);
				} else {
					searchState = SearchState.Default;
				}
				
				break;
				
			case Iddle:
			
				// iddle state, waiting for capture to return and stack block
				try{
					Thread.sleep(Util.SLEEP_PERIOD);
				} catch(Exception e) {}
				break;
			}
		}
		
	}

	// travel to correspondent axis of current cardinal point
	private void travelToAxis(boolean frontwards) {
		
		String axis = (currCardinal % 2 == 0) ? "Y" : "X";
	
		odo.setMotorSpeeds(Util.MOTOR_FAST, Util.MOTOR_FAST);
		if (frontwards) odo.forwardMotors();
		else odo.backwardMotors();
		
		switch(axis){
		case "X": 
			while(odo.getX() < cardinals[currCardinal][0] - Util.CM_TOLERANCE || 
					odo.getX() > cardinals[currCardinal][0] + Util.CM_TOLERANCE);
			break;
		case "Y": 
			while(odo.getY() < cardinals[currCardinal][1] - Util.CM_TOLERANCE ||
					odo.getY() > cardinals[currCardinal][1] + Util.CM_TOLERANCE);
			break;
		}
		
		odo.stopMotors();
		
	}

	/**
	 * When object is in sight, approach slowly and inspect
	 */
	private void inspectObject(double X, double Y) {
		odo.setMotorSpeed(Util.MOTOR_SLOW); 
		odo.forwardMotors(); 
		
		//wait until close enough to determine if it's a styrofoam block
		while(usSensor.getMedianSample(Util.US_SAMPLES) > Util.BLOCK_DISTANCE ||
				Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, new double[] {X,Y}) < Util.BLOCK_DISTANCE); 
		odo.stopMotors();
		
		// inspect object
		if(isStyrofoamBlock()) { 
			searchState = SearchState.Iddle;
			Main.state = Main.RobotState.Capture;
		} else {
			// backoff a little to avoid hitting the block while turning
			odo.setMotorSpeed(Util.MOTOR_SLOW);
			odo.backwardMotors();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			// return to cardinal point and inspect next object
			nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
			searchState = SearchState.Default;
		}	
	}

	/**
	 * 
	 * @return if the detected object is a styrofoam block
	 */
	//TODO use complete RGB vector to compare
	private boolean isStyrofoamBlock() {
		return (colorSensor.getColor()[0] < colorSensor.getColor()[1]);
	}
	
	/**
	 * 
	 * @return if there is an object that can be detected by the robot
	 */
	private boolean isObjectDetected() {
	
		float currDistance = usSensor.getMedianSample(Util.US_SAMPLES);	
		double currHeading = odo.getTheta();
		double objX =  odo.getX() + currDistance*Math.cos(currHeading);
		double objY = odo.getY() + currDistance*Math.sin(currHeading);
		
		// always latch first encountered object at each cardinal
		if (this.lastDistanceDetected < 0)  {
			// at each iteration of scatter search, last distance is reset
			this.lastDistanceDetected = currDistance;
			this.lastX = objX; 
			this.lastY = objY;
			return true;
		}
		

		
		// avoids latching the same object multiple times 
		boolean isObject = (Odometer.euclideanDistance(new double[] {this.lastX, this.lastY},
								new double[] {objX, objY}) > 5);;
		//TODO if distance between last latched point and current > foam side
//		boolean isObject = (currentDistance < lastDistanceDetected + Util.SEARCH_DISTANCE) && 
//						(currentDistance > lastDistanceDetected + Util.SEARCH_DISTANCE) && (currentDistance <= Util.SEARCH_DISTANCE);
		lastDistanceDetected = currDistance;
		
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
