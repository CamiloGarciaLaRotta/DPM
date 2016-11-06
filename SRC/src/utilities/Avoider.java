package utilities;

import java.awt.Rectangle;

import chassis.USSensor;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * @version 0.1
 * @author juliette
 * Checks if the robot must avoid an object in its path and then does it
 */
public class Avoider extends Thread{
	
	private Odometer odo;
	private Navigation nav;
	private USSensor usSensor;
	
	private double[] currPos = new double[3];
	private double distance;
	
	private Rectangle RED;
	private Rectangle currRect = new Rectangle();
	
	// corners
	private final Rectangle x1 = new Rectangle(-5,-5,10, 10);
	private final Rectangle x2 = new Rectangle(300,-5,10, 10);
	private final Rectangle x3 = new Rectangle(300,300,10, 10);
	private final Rectangle x4 = new Rectangle(-5,300,10, 10);
	
	public enum AvoidState {Disabled, Enabled};
	public static AvoidState avoidState = AvoidState.Disabled;
	
	//TODO TODO TODO TODO
	// - Search requires this thread to change its state once its safe to continue
	// - Handle CW and CCW avoidance depending on the position of the robot on the map
	// - Right now avoidance only avoids blocks or zones one at a time, make dynamic
	// - Discuss with Software if use of java.awt.rectangle is pertinent
	// - Is it alright to hardcode corner values? they are absolute and final regardless anything
	
	public Avoider(Odometer odo, Navigation nav, USSensor usSensor) {
		this.odo = odo;
		this.nav = nav;
		this.usSensor = usSensor;
		int red_width = 1;	// to be updated via constructor 
		int red_height = 2;	// depending on wifi RED coordinates
		this.RED = new Rectangle(0*40,9*40,red_width*40, red_height*40);
	}

	@Override
	public void run() {
		while(true){
			
			distance = usSensor.getMedianSample(Util.US_SAMPLES);
			
			// build current position rectangle
			odo.getPosition(this.currPos);
			currRect.setBounds((int)(currPos[0]*Util.SQUARE_LENGTH), (int)(currPos[1]*Util.SQUARE_LENGTH),5, 5);
			
			boolean CCW = chooseOrientation(currPos);
			
			if(Avoider.avoidState == AvoidState.Enabled) {			
				
				// check for physical obstacles
				if(distance < Util.MIN_D){
					odo.stopMotors();
					
					do{
						linearAvoidance(CCW);
					} while (usSensor.getMedianSample(Util.US_SAMPLES) < Util.MIN_D);
					
					// no more obstacle ahead, advance a bit before returning control to Search
					odo.moveCM(LINEDIR.Forward, Util.WOOD_MIN_WIDTH, true);
				}
				
				// check for RED zone
				if(RED.contains(currRect)){
					do{
						linearAvoidance(CCW);
					} while (RED.contains(currRect));
				}
				
				// check for corners zones
				if(x1.contains(currRect) || x2.contains(currRect) ||
					x3.contains(currRect) || x4.contains(currRect)){
					// return to GREEN, nothing to do in a corner
					Search.searchState = SearchState.Default;
				}
			}
			
			// lower stress on CPU
			try {
				Thread.sleep(Util.AVOIDER_PERIOD);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	// choose rotation sense depending on current position and heading
	private boolean chooseOrientation(double[] currPos) {
		//TODO
		return false;
	}

	// linear set of instructions to avoid obstacle
	private void linearAvoidance(boolean CCW) {
		int coeff = (CCW) ? -1 : 1;
		nav.turnBy(coeff*90);
		odo.moveCM(LINEDIR.Forward, Util.WOOD_MIN_WIDTH, true);
		nav.turnBy(-1*coeff*90);
	}
}
