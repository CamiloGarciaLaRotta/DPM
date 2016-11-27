package utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import chassis.ColorSensor;
import chassis.Main;
import chassis.Main.RobotState;
import lejos.hardware.Sound;
import chassis.USSensor;
import utilities.Avoider.AvoidState;
import utilities.Capture.CaptureState;
import utilities.Odometer.LINEDIR;
import utilities.Odometer.TURNDIR;

/**
 * Blue styrofoam block search functionality for the robot
 * Goes to cardinal points on the field and searches for blocks at each
 * @author juliette
 * @version 3.0
 */
public class Search extends Thread {
	
	// Instances
	private Odometer odo;	
	private Navigation nav;
	private USSensor usSensor;
	private static ColorSensor colorSensor;
	
	// Coordinates
	private double[] N = new double[2];
	private double[] W = new double[2];
	private double[] S = new double[2];
	private double[] E = new double[2];
	public static double[][] cardinals = new double[4][2];
	public int currCardinal;
	
	// object detection
	private ArrayList<double[]> objectLocations = new ArrayList<double[]>();
	private float lastDistanceDetected;
	private double lastX, lastY;
	
	// states
	public enum SearchState {Default, AtCardinal, AtDropZone, Inspecting, Idle};
	public static SearchState searchState;
	
	// TODO TODO TODO TODO
	// - setter for GREEN/RED on all classes
	
	/**
	 * Constructor for Search Class
	 * @param odometer Odometer object
	 * @param colorSensor ColorSensor object
	 * @param usSensor USSensor object
	 * @param GREEN coordinates of the green scoring zone
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
		
		double diffX = GREEN[1][0] - GREEN[0][0];
		double diffY = GREEN[1][1] - GREEN[0][1];
		
		double paddingX = 0;
		double paddingY = 0;
		
		// make cardinal point further from the tower position
		if (diffX == diffY) {
			paddingX = Util.ROBOT_LENGTH/2;
			paddingY = Util.ROBOT_LENGTH/2;
		} else if (diffX > diffY) {
			paddingY = Util.ROBOT_LENGTH/2;
		} else {
			paddingX = Util.ROBOT_LENGTH/2;
		}
		
		// cardinal search points
		this.S = new double[] {midX, GREEN[0][1]-paddingY};
		this.N = new double[] {midX, GREEN[1][1]+paddingY};
		this.W = new double[] {GREEN[0][0]-paddingX, midY};
		this.E = new double[] {GREEN[1][0]+paddingX, midY};
		
		this.cardinals = new double[][] {N,W,S,E};
		
		this.lastDistanceDetected = -1;
		this.currCardinal = -1;
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
			try {Thread.sleep(Util.SLEEP_PERIOD);} catch (Exception ex) {}
		}
		
		isStyrofoamBlock(); //Initialize rgb mode
		
		while(true){
			if(Main.state == RobotState.Avoiding) Search.searchState = SearchState.Idle;
			
			switch(searchState) {
			
			case Default: 
				// at origin, not yet at GREEN zone
				if(currCardinal == -1) { 
					
					// next cardinal, wrap around
					currCardinal++; currCardinal %= 4;
					
					// until it arrives at current cardinal
					while(Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, 
									new double[] {cardinals[currCardinal][0], cardinals[currCardinal][1]}) > Util.TRAVELTO_BW){ 
						
						double targetHeading = Math.atan2(cardinals[currCardinal][1] - odo.getY(), cardinals[currCardinal][0] - odo.getX());

						odo.setMotorSpeed(USLocalizer.ROTATION_SPEED);
						// will always be CW because localization will always end with the robot with the wall on its left
						odo.spin(Odometer.TURNDIR.CW);
						double minHeading = odo.getTheta();
						double minDistance = Main.usSensor.getMedianSample(Util.US_SAMPLES);
						double distance;
						while(Math.abs(Navigation.minimalAngle(odo.getTheta(), targetHeading)) > Util.SCAN_THETA_THRESHOLD) {
							if((distance = Main.usSensor.getMedianSample(Util.US_SAMPLES)) < minDistance){ 
								minDistance = distance;
								minHeading = odo.getTheta();
							}
						}
						odo.stopMotors();
						
						//just to make sure we get the last heading
						if((distance = Main.usSensor.getMedianSample(Util.US_SAMPLES)) < minDistance){ 
							minDistance = distance;
							minHeading = odo.getTheta();
						}
						
						// if there's an approachable block in sight
						if(minDistance < Util.SEARCH_DISTANCE) {
//							Sound.twoBeeps();
							nav.travelTo(minDistance * Math.cos(minHeading) + odo.getX(), minDistance * Math.sin(minHeading) + odo.getY());
						}
						else nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
						
						// verify if navigation was interrupted
						if (Navigation.PathBlocked) {
							// is the block a target or an obstacle?
							if(testForStyrofoam()) {
								searchState = SearchState.Idle;
								Main.state = RobotState.Capture;
								Capture.setContext(cardinals[currCardinal]);
								Capture.captureState = CaptureState.Grab;
								while(Main.state != RobotState.Search) {
									try { Thread.sleep(2*Util.SLEEP_PERIOD); } catch(Exception ex) {}
								}
								//searchState = SearchState.Default;
							}
							else {
								Main.forklift.liftUp();
								odo.moveCM(Odometer.LINEDIR.Backward,Util.BACKUP_DISTANCE,true);
								Avoider.avoidState = AvoidState.Enabled;
								// wait for avoider to finish
								//avoid race condition
								try {Thread.sleep(2*Util.SLEEP_PERIOD);} catch (Exception ex) {}
								while(Main.state == RobotState.Avoiding) {
									try{Thread.sleep(Util.SLEEP_PERIOD);}catch(Exception ex) {}
								}
								//Avoider.avoidState = AvoidState.Disabled;
							}
						}	
					}
					
					Search.searchState = SearchState.AtCardinal;
					
				} else { 
					
					// no more object at that cardinal point
					if(this.objectLocations.isEmpty()) {
						// next cardinal, wrap around
						currCardinal++; currCardinal %= 4;
//						Sound.beepSequence();
						
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
				if(	cardinals[currCardinal][0] < 0.0 || cardinals[currCardinal][1] > 10.0 * Util.SQUARE_LENGTH ||
					cardinals[currCardinal][1] < 0.0 || cardinals[currCardinal][1] > 10.0 * Util.SQUARE_LENGTH) 
				{
					Search.searchState = SearchState.Default;
					break;
				}

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
					testForObject();
					// resume scatter search
					odo.setMotorSpeed(Util.MOTOR_SLOW);
					odo.spin(TURNDIR.CCW);
				}
				
				odo.stopMotors();
				
				// sort list of object locations
				Collections.sort(this.objectLocations, new Comparator<double[]>() {
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
				odo.moveCM(LINEDIR.Backward, Util.ROBOT_WIDTH/2, true);
				
				nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
				
				searchState = SearchState.Default;
				
				break;
				
			case Inspecting: 
				
				if(!this.objectLocations.isEmpty()) {
					// examine closest object
					
					double X = this.objectLocations.get(0)[0];
					double Y = this.objectLocations.get(0)[1];
					
					// turn towards object
					nav.turnTo(this.objectLocations.get(0)[3], true); 
					
					this.objectLocations.remove(0);
					
					inspectObject(X, Y);
					if(Main.state == RobotState.Capture) continue;
					Main.forklift.liftUp();
				} else {
					searchState = SearchState.Default;
				}
				
				break;
				
			case Idle:
			
				// idle state, waiting for capture or avoider to return
				try {Thread.sleep(Util.SLEEP_PERIOD);} catch (Exception ex) {}
				break;
			}
		}
		
	}

	/**
	 * Approach an object to verify its nature
	 * @return if the object is a styrofoam block
	 */
	private boolean testForStyrofoam() {
		odo.setMotorSpeed(Util.MOTOR_SLOW);
		odo.forwardMotors();
		while(Main.usSensor.getMedianSample(Util.US_SAMPLES) > Util.BLOCK_DISTANCE);
		odo.stopMotors();
		Main.forklift.liftDown();
		
		boolean styrofoam = isStyrofoamBlock();
		
		return styrofoam;
	}
	/**
	 * Checks if there is an object that can be detected by the robot
	 * @return if there is an object that can be detected by the robot
	 */
	private boolean testForObject() {
		
		// object detected
		odo.stopMotors();
		
		float currDistance = usSensor.getMedianSample(Util.US_SAMPLES);
		double currHeading = odo.getTheta();
		double objX =  odo.getX() + currDistance*Math.cos(currHeading);
		double objY = odo.getY() + currDistance*Math.sin(currHeading);
		
		// no object detected
		if (currDistance > Util.SEARCH_DISTANCE) return false;
		
		// make sure object in sight is not wall
		if(objY <= Util.SOUTH_MAX || objY >= Util.NORTH_MAX || 
				objX <= Util.WEST_MAX || objX >= Util.EAST_MAX) {
			return false;
		}
		
		boolean isObject;
		
		// always latch first encountered object at each cardinal
		if (this.lastDistanceDetected < 0)  {
			// at each iteration of scatter search, last distance is reset
			this.lastX = objX; 
			this.lastY = objY;
			isObject = true;
		} else {
			// avoids latching the same object multiple times 
			 isObject = (Odometer.euclideanDistance(new double[] {this.lastX, this.lastY},
									new double[] {objX, objY}) > Util.WOOD_MIN_WIDTH);
		}
		
		// latch all important information
		if (isObject) {
			this.objectLocations.add(new double[] {objX,objY,currDistance,currHeading});
			this.lastDistanceDetected = currDistance;
			this.lastX = objX;
			this.lastY = objY;
		}
		
		return isObject;
	}

	/**
	 * When object is in sight, approach slowly and inspect
	 */
	private void inspectObject(double X, double Y) {
		odo.setMotorSpeed(Util.MOTOR_SLOW); 
		odo.forwardMotors(); 
		
		//wait until close enough to determine if it's a styrofoam block
		while(usSensor.getMedianSample(Util.US_SAMPLES) > Util.BLOCK_DISTANCE &&
				Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, new double[] {X,Y}) > Util.BLOCK_DISTANCE); 
		odo.stopMotors();
		
		//Scan left and right to find minimum distance to object. This should reduce the frequency of the robot detecting the block at a bad angle
		FOV(Util.SEARCH_FOV);
		
		// inspect object
		if(isStyrofoamBlock()) { 
			searchState = SearchState.Idle;
			Main.state = Main.RobotState.Capture;
			Capture.captureState = CaptureState.Grab;
			Capture.setContext(cardinals[currCardinal]);
			return;
		} else {
			Main.forklift.liftUp();
			odo.moveCM(LINEDIR.Backward, 5, true);
		} 
		
		nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
		searchState = SearchState.Default;
	}
	
	/**
	 *  Perform a swipe scan of a certain aperture angle to precisely identify block
	 *  At this stage the robot is ensured to be in close proximity of block
	 * @param angle the field of view (FOV) to scan
	 */
	protected void FOV(double angle) {
		double minDistance = usSensor.getMedianSample(Util.US_SAMPLES);
		double minHeading = odo.getTheta();
		double ccwHeading = odo.getTheta() + angle/2;
		double cwHeading = odo.getTheta() - angle/2;
		odo.setMotorSpeed(Odometer.ROTATE_SPEED);
		//Rotate counterclockwise by angle/2
		odo.spin(Odometer.TURNDIR.CCW);
		while(Math.abs(Navigation.minimalAngle(odo.getTheta(), ccwHeading)) > Util.SCAN_THETA_THRESHOLD) {
			if(usSensor.getMedianSample(Util.US_SAMPLES) < minDistance)  {
				minHeading = odo.getTheta();
				minDistance = usSensor.getMedianSample(Util.US_SAMPLES);
			}
		}
		odo.stopMotors();
		odo.setMotorSpeed(Odometer.ROTATE_SPEED);
		//Rotate clockwise by angle/2
		odo.spin(Odometer.TURNDIR.CW);
		while(Math.abs(Navigation.minimalAngle(odo.getTheta(), cwHeading)) > Util.SCAN_THETA_THRESHOLD) {
			if(usSensor.getMedianSample(Util.US_SAMPLES) < minDistance)  {
				minHeading = odo.getTheta();
				minDistance = usSensor.getMedianSample(Util.US_SAMPLES);
			}
		}
		
		nav.turnTo(minHeading, true);
		if(minDistance < Util.SEARCH_DISTANCE) {	//if the minimum distance is still far away, will assume it is nothing and won't go to it
			odo.moveCM(Odometer.LINEDIR.Backward, Util.BLOCK_DISTANCE - minDistance, true);
			Main.forklift.liftDown();
		}
	}
	

	/**
	 * Travel to the corresponding axis
	 * Axis based on current cardinal point.
	 * @param frontwards direction to move in relative to robot
	 */
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
	 * Checks if the oject in front of the robot is a styrofoam block
	 * @return if the detected object is a styrofoam block
	 */
	protected static boolean isStyrofoamBlock() {
//		return (colorSensor.getColor()[0] < colorSensor.getColor()[1]);
		boolean isStyrofoam;
		
		float[] measuredRGB = Main.colorSensor.getColor();
		//get unit vector
		double magnitude = Math.sqrt(
				(double)(measuredRGB[0]*measuredRGB[0]) + (double)(measuredRGB[1]*measuredRGB[1]) + (double)(measuredRGB[2]*measuredRGB[2]));
		double[] normRGB = new double[3];
		normRGB[0] = (double)measuredRGB[0]/magnitude;
		normRGB[1] = (double)measuredRGB[1]/magnitude;
		normRGB[2] = (double)measuredRGB[2]/magnitude;
		
		//compare measurement to standard for styrofoam block with a dot product
		if(normRGB[0]*Util.FOAM_RGB_VECTOR[0] + normRGB[1]*Util.FOAM_RGB_VECTOR[1] + normRGB[2]*Util.FOAM_RGB_VECTOR[2] > Util.VECTOR_TOLERANCE) {
			isStyrofoam = true;
		} else {
			isStyrofoam = false;
		}
		return isStyrofoam;
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
