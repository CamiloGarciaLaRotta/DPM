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
 * @version 0.2
 * @author juliette

 */
public class Avoider extends Thread{
	
	// instances
	private Odometer odo;
	private Navigation nav;
	private USSensor usSensor;
	
	// coordinates
	private double[] currPos = new double[3];
	private Rectangle RED;
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
		int red_width = (int)(RED[1][0] - RED[0][0]); 
		int red_height = (int)(RED[1][1] - RED[0][1]);	
		this.RED = new Rectangle((int)(RED[0][0]*40), (int)(RED[1][1]*40), red_width*40, red_height*40);
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
				//}
			}
			
			// check for RED zone
			if(RED.contains(currRect)){
				Main.state = RobotState.Avoiding;
				odo.stopMotors();
				// determine in which thread the robot was acting 
				if(Capture.captureState != CaptureState.Iddle){
					Capture.captureState = CaptureState.Iddle;
				} else {
					Search.searchState = SearchState.Iddle; 
				}
				redAvoidance();	
				do{
										
				} while (RED.contains(currRect));
			}
			
			// check for corners zones
			if(x1.contains(currRect) || x2.contains(currRect) ||
				x3.contains(currRect) || x4.contains(currRect)){
				// return to GREEN, nothing to do in a corner
				Search.searchState = SearchState.Default;
				Main.state = RobotState.Search;
				Capture.captureState = CaptureState.Iddle;
				continue;
			}
			
			if(lastCaptureState != CaptureState.Iddle) {
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
		double Heading = this.currPos[2];
		
		//TODO
		
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
		nav.turnBy(coeff*90);
		nav.travelTo(odo.getX() + Math.cos(odo.getTheta()) * Util.WOOD_MIN_WIDTH, odo.getY() + Math.sin(odo.getTheta()) * Util.WOOD_MIN_WIDTH);
		double[] pos = new double[3];
		odo.getPosition(pos);
		if(Navigation.PathBlocked) linearAvoidance(CCW); //Recursively avoid obstacles if there's an obstacle in the avoidance path
		nav.turnBy(-1*coeff*90);
	}
}
