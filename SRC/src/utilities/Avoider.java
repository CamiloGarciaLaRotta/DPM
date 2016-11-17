package utilities;

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
	
	private double distance;

	
	//TODO TODO TODO TODO
	// - find more elegant way of choosing CW or CCW for obstacle avoidance
	// - Discuss with Software if use of java.awt.rectangle is pertinent
	// - Is it alright to hardcode corner values? they are absolute and final regardless anything
	
	/**
	 * 
	 * @param odo - Odometer object
	 * @param nav - Navigation object
	 * @param usSensor - USSensor object
	 * @param RED - coordinates of red zone to always avoid
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
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(true){
			
			CaptureState lastCaptureState = Capture.captureState;
			SearchState lastSearchState = Search.searchState;
			
			distance = usSensor.getMedianSample(Util.US_SAMPLES);
			
			// build current position rectangle
			odo.getPosition(this.currPos);
			currRect.setBounds((int)(currPos[0]*Util.SQUARE_LENGTH), (int)(currPos[1]*Util.SQUARE_LENGTH), Util.ROBOT_WIDTH, Util.ROBOT_LENGTH);
			
			boolean CCW = chooseOrientation(currPos);
			
			if(Avoider.avoidState == AvoidState.Enabled) {			
				
				Main.state = RobotState.Avoiding;
				
				// check for physical obstacles
				//if(distance < Util.AVOID_DISTANCE){
					odo.stopMotors();
					
					do{
						linearAvoidance(CCW);
					} while (usSensor.getMedianSample(Util.US_SAMPLES) < Util.AVOID_DISTANCE);
					
					// no more obstacle ahead, advance a bit before returning control to Search
					odo.moveCM(LINEDIR.Forward, Util.ROBOT_WIDTH, true);
					if(lastCaptureState != CaptureState.Idle) Main.state = RobotState.Capture;
					else Main.state = RobotState.Search;
					Capture.captureState = lastCaptureState;
					Search.searchState = lastSearchState;
					avoidState = AvoidState.Disabled;
				//}
			}
			
			// check for RED zone
			if(RED.contains(currRect)){
				Main.state = RobotState.Avoiding;
				odo.stopMotors();
				// determine in which thread the robot was acting 
				if(Capture.captureState != CaptureState.Idle){
					Capture.captureState = CaptureState.Idle;
				} else {
					Search.searchState = SearchState.Idle; 
				}
				redAvoidance();	
				do{
										
				} while (RED.contains(currRect));
			}
			
			// check for corners zones
			if(Search.searchState != SearchState.Default && (x1.contains(currRect) || x2.contains(currRect) ||
				x3.contains(currRect) || x4.contains(currRect))){
				// return to GREEN, nothing to do in a corner
				Search.searchState = SearchState.Default;
				Main.state = RobotState.Search;
				Capture.captureState = CaptureState.Idle;
				continue;
			}
			
			if(lastCaptureState != CaptureState.Idle) {
				Main.state = RobotState.Capture;
				Capture.captureState = lastCaptureState;
			}
			else {
				Main.state = RobotState.Search;
				Search.searchState = lastSearchState;
			}
			
			
			// lower stress on CPU
			try {
				Thread.sleep(Util.SLEEP_PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	// special set of instructions required to avoid RED zone
	private void redAvoidance() {
		double currX = this.currPos[0];
		double currY = this.currPos[1];
		
		int currCardinal = 0;
		double distanceToNextAxis = 0;
		
		// TODO HANDLE SPECIAL CASE -> WHAT IF ROBOT ARRIVES AT CORNER
		
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
		for(int i = 0; i < 2; i++){
			currCardinal++; currCardinal %= 4;
			nav.turnTo(currCardinal*Math.PI/2 + Math.PI/2, true);
			distanceToNextAxis = (currCardinal % 2 == 0) ? Math.abs(currY-red_cardinals[currCardinal]) :
															Math.abs(currX-red_cardinals[currCardinal]);
			odo.moveCM(LINEDIR.Forward, distanceToNextAxis , true);
		}	
		
		// face away from the RED zone
		nav.turnTo(currCardinal*Math.PI/2, true);
	}

	/**
	 * Choose rotation sense depending on current position and heading
	 * Decides direction to turn when object encountered
	 * @param currPos - X, Y, heading coordinates
	 * @return - true is counterclockwise, false is clockwise
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
	 *  linear set of instructions to avoid obstacle
	 * @param CCW - direction to turn (should be determined by chooseOrientation)
	 */
	private void linearAvoidance(boolean CCW) {
		int coeff = (CCW) ? -1 : 1;
		nav.turnBy(coeff*Math.PI/2);
		nav.travelTo(odo.getX() + Math.cos(odo.getTheta()) * (Util.WOOD_MIN_WIDTH + Util.TRACK/2), odo.getY() + Math.sin(odo.getTheta()) * (Util.WOOD_MIN_WIDTH + Util.TRACK/2));
		double[] pos = new double[3];
		odo.getPosition(pos);
		if(Navigation.PathBlocked) linearAvoidance(CCW); //Recursively avoid obstacles if there's an obstacle in the avoidance path
		nav.turnBy(-1*coeff*Math.PI/2);
	}
}
