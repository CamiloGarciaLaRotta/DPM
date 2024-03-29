package utilities;

/*
 * AUTHORS
 * Camilo Garcia La Rotta
 */

import java.awt.Rectangle;

import chassis.Main;
import chassis.USSensor;
import chassis.Main.RobotState;
import utilities.Capture.CaptureState;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * Checks if the robot must avoid an object in its path and then does it
 * Runs constantly and can interrupt other processes.
 * @version 3.0
 */
public class Avoider extends Thread{
	
	// instances
	private Odometer odo;
	private Navigation nav;
	private USSensor usSensor;
	
	// coordinates
	private double[] currPos = new double[3];
	private Rectangle RED;
	private double RED_N, RED_S, RED_W, RED_E;
	private double[] red_cardinals = new double[4];
	private Rectangle currRect = new Rectangle();
	
	// corners
	private final Rectangle x1 = new Rectangle(-5,-5,10, 10);
	private final Rectangle x2 = new Rectangle(300,-5,10, 10);
	private final Rectangle x3 = new Rectangle(300,300,10, 10);
	private final Rectangle x4 = new Rectangle(-5,300,10, 10);
	
	// states
	public enum AvoidState {Disabled, Enabled};
	public static AvoidState avoidState = AvoidState.Disabled;
	

	
	/**
	 * Avoider constructor
	 * @param odo Odometer object
	 * @param nav Navigation object
	 * @param usSensor USSensor object
	 * @param RED coordinates of red zone to always avoid
	 */
	public Avoider(Odometer odo, Navigation nav, USSensor usSensor, double[][] RED) {
		this.odo = odo;
		this.nav = nav;
		this.usSensor = usSensor;	
		
		// X, Y axis which mark the borders of the RED zone
		this.RED_N = RED[1][1];
		this.RED_S = RED[0][1];
		this.RED_W = RED[0][0];
		this.RED_E = RED[1][0];
		
		this.red_cardinals = new double[] {RED_N,RED_W,RED_S,RED_E};
		
		int red_width = (int)(this.RED_E - this.RED_W); 
		int red_height = (int)(this.RED_N - this.RED_S);
		
		// RED awt.rectangle
		this.RED = new Rectangle((int)(this.RED_W*40), (int)(this.RED_N*40), red_width*40, red_height*40);
	}

	/**
	 * Execution for robot avoidance. Runs constantly regardless of current state.
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(true){
			//Save the last states of Capture and Search, so we can set them to Idle temporarily during Avoider operation
			CaptureState lastCaptureState = Capture.captureState; 
			SearchState lastSearchState = Search.searchState;
			
			//Save last robot state
			int lastRobotState = Main.state.ordinal();
						
			// build current position rectangle
			odo.getPosition(this.currPos);
			currRect.setBounds((int)(currPos[0]*Util.SQUARE_LENGTH), (int)(currPos[1]*Util.SQUARE_LENGTH), Util.ROBOT_WIDTH, Util.ROBOT_LENGTH);
			
			boolean CCW = chooseOrientation(currPos); //Chooses if it is more appropriate to turn CCW or CW
			
			if(Avoider.avoidState == AvoidState.Enabled) {
				
				if(Main.state != RobotState.Finished) Main.state = RobotState.Avoiding;
				odo.stopMotors();
				
				do{
					linearAvoidance(CCW);
				} while (usSensor.getMedianSample(Util.US_SAMPLES) < Util.AVOID_DISTANCE);
				
				//check to make sure wheels won't swipe the obstacle
				double initialHeading = odo.getTheta();
				//distance robot avoids to and distance from CoR to outer bound
				double absoluteTurnAngle = initialHeading - Math.atan2(Util.AVOID_DISTANCE, Util.ROBOT_WIDTH/2);
				boolean pathClear = true;
				odo.setMotorSpeed(USLocalizer.ROTATION_SPEED);
				odo.spin(CCW ? Odometer.TURNDIR.CW : Odometer.TURNDIR.CCW);   //spin in direction opposite of linearAvoidance
				while(Math.abs(odo.getTheta() - absoluteTurnAngle) > Util.DEG_TOLERANCE) {
					if(usSensor.getMedianSample(Util.US_SAMPLES) < Util.AVOID_DISTANCE) pathClear = false;
				}
				odo.stopMotors();
				nav.turnTo(initialHeading, true);
				
				if(pathClear) {
					// no more obstacle ahead, advance a bit before returning control to Search
					odo.moveCM(LINEDIR.Forward, Util.ROBOT_LENGTH, true);
	
					if(Main.state != RobotState.Finished) {
						if(lastCaptureState != CaptureState.Idle) Main.state = RobotState.Capture;
						else if(lastSearchState != SearchState.Idle) Main.state = RobotState.Search;
						else Main.state = RobotState.values()[lastRobotState];
						Capture.captureState = lastCaptureState;
						Search.searchState = lastSearchState;
						avoidState = AvoidState.Disabled;
					}
				}
			}
			
			// check for RED zone
			if(RED.contains(currRect)){
				odo.stopMotors();
				if(Main.state != RobotState.Finished) { 
					Main.state = RobotState.Avoiding;
					// determine in which thread the robot was acting 
					if(Capture.captureState != CaptureState.Idle){
						Capture.captureState = CaptureState.Idle;
					} else {
						Search.searchState = SearchState.Idle; 
					}
				}
				redAvoidance();	
			}
			
			// check for corners zones
			if(Main.state != RobotState.Finished && Search.searchState != SearchState.Default &&
					(x1.contains(currRect) || x2.contains(currRect) || x3.contains(currRect) || x4.contains(currRect))){
				// return to GREEN, nothing to do in a corner
				Search.searchState = SearchState.Default;
				Main.state = RobotState.Search;
				Capture.captureState = CaptureState.Idle;
				continue;
			}
			
			if(Main.state != RobotState.Finished) {
				//Reset Search and Capture to appropriate states
				if(lastCaptureState != CaptureState.Idle) {
					Main.state = RobotState.Capture;
					Capture.captureState = lastCaptureState;
				}
				else if(lastSearchState != SearchState.Idle) {
					Main.state = RobotState.Search;
					Search.searchState = lastSearchState;
				}
				else {
					Main.state = RobotState.values()[lastRobotState];
				}
			}
			
			
			// lower stress on CPU
			try {
				Thread.sleep(Util.SLEEP_PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Special set of instructions to avoid the RED zone
	 */
	private void redAvoidance() {
		double currX = this.currPos[0];
		double currY = this.currPos[1];
		
		int currCardinal = 0;
		double distanceToNextAxis = 0;
		
		// identify which side of RED zone we are facing
		if(currY < this.RED_N+5 && currY > this.RED_N-5){
			currCardinal = 0;
		} else if(currX < this.RED_W+5 && currX > this.RED_W-5){
			currCardinal = 1;
		} else if(currY < this.RED_S+5 && currX > this.RED_S-5){
			currCardinal = 2;
		} else if(currX < this.RED_E+5 && currX > this.RED_E-5){
			currCardinal = 3;
		}
		
		// go around it (by default CCW)
		for(int i = 0; i < 1; i++){
			currCardinal++; currCardinal %= 4;
			nav.turnTo(currCardinal*Math.PI/2 + Math.PI/2, true);
			distanceToNextAxis = (currCardinal % 2 == 0) ? Math.abs(currY-red_cardinals[currCardinal]) :
															Math.abs(currX-red_cardinals[currCardinal]);
			odo.moveCM(LINEDIR.Forward, distanceToNextAxis , true);
		}	
		
		// at this position the robot is on the opposite end of the RED rectangle, 
		// it can safely continue its path
	}

	/**
	 * Choose rotation sense depending on current position and heading
	 * Decides direction to turn when object encountered
	 * @param currPos X, Y, heading coordinates
	 * @return chosen rotation sense - true is counterclockwise, false is clockwise
	 */
	private boolean chooseOrientation(double[] currPos) {
		double currX = currPos[0];
		double currY = currPos[1];
		double currT = currPos[2];
		
		if(currT  < 0.0) currT += 2*Math.PI;
		else if(currT > 2*Math.PI) currT -= 2 * Math.PI;
		
		if(currT > Math.PI/4 && currT < 3*Math.PI/4) {
			if (currX > 8*Util.SQUARE_LENGTH) return true;
		} else if (currT > 5*Math.PI/4 && currT < 7*Math.PI/4) {
			if (currX < 2*Util.SQUARE_LENGTH) return true;
		} else if (currT > 3*Math.PI/4 && currT < 5*Math.PI/4) {
			if (currY > 8*Util.SQUARE_LENGTH) return true;
		} else if (currT > 7*Math.PI/4 || currT < Math.PI/4) {
			if (currY < 2*Util.SQUARE_LENGTH) return true;
		}
		
		return false;
	}

	/**
	 * Linear set of instructions to avoid obstacle
	 * Runs recursively if more obstacles are encountered during avoidance
	 * @param CCW direction to turn (should be determined by chooseOrientation)
	 */
	private void linearAvoidance(boolean CCW) {
		int coeff = (CCW) ? -1 : 1;
		nav.turnBy(coeff*Math.PI/2);
		nav.travelTo(odo.getX() + Math.cos(odo.getTheta()) * (Util.WOOD_MIN_WIDTH + Util.TRACK), odo.getY() + Math.sin(odo.getTheta()) * (Util.WOOD_MIN_WIDTH + Util.TRACK),false);
		double[] pos = new double[3];
		odo.getPosition(pos);
		if(Navigation.PathBlocked) linearAvoidance(CCW); //Recursively avoid obstacles if there's an obstacle in the avoidance path
		nav.turnBy(-1*coeff*Math.PI/2);
	}
}
