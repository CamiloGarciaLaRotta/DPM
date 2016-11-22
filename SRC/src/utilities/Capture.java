package utilities;

import chassis.Main;
import chassis.Main.RobotState;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * Thread that runs when capturing a styrofoam block
 * @version 3.0
 * 
 *
 */
public class Capture extends Thread {
	
	// instances
	private Odometer odo;
	private Navigation nav;
	private Search search;
	
	// coordinates
	private double[][] GREEN;
	private double[] towerPosition;
	
	private static double[] cardinalPoint;
	
	// states
	public enum CaptureState {Grab, Return, Stack, Idle};
	public static CaptureState captureState = CaptureState.Idle;

	private EV3LargeRegulatedMotor clawMotor;
	
	private int towerHeight;
	
	/**
	 * Capture Thread Constructor
	 * @param odometer Odometer Object
	 * @param GREEN green scoring zone coordinates
	 */
	public Capture(Odometer odometer, Search search, double[][] GREEN) {
		this.odo = odometer;
		this.nav = new Navigation(this.odo);
		this.search = search;
		this.GREEN = GREEN;
		this.towerHeight = 0;
		this.towerPosition = new double[]{(GREEN[0][0] + GREEN[1][0])/2,(GREEN[0][1] + GREEN[1][1])/2};
	}
	
	/**
	 * start Capture thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(true) {
			if(Main.state == RobotState.Avoiding) Capture.captureState = CaptureState.Idle;
			
			switch(captureState) {
			case Grab:
				//TODO: Make sure block is in range
				Main.forklift.liftDown();
				odo.moveCM(Odometer.LINEDIR.Forward, Util.APPROACH_BLOCK, true);
				Main.forklift.grip();
				Main.forklift.liftUp();
				captureState = CaptureState.Return;
				break;
			case Return:
				// Thread to verify claw still has block
				/*
				(new Thread() {
					  public void run() {
						  // only active during the return state
					    while(Capture.captureState == CaptureState.Return){
					    	// obtain color samples
					    	int negatives = 0;
					    	for(int i = 0; i < 10*Util.US_SAMPLES; i++){
					    		// TODO #TESTING as of now I check if the colorsensor sees a RGB[0] > 0.01
					    		// as it was the value i got while dryrunning. I leave it to y'all to choose the
					    		// RGB threshod that works best to detect when the block is no longuer in the claw
					    		if(Main.colorSensor.getColor()[0] < Util.FOAM_RGB_VECTOR[0]-Util.COLOR_BW) negatives++;	
					    	}
					    	// more than half negative samples => lost block 
					    	if (negatives > Util.US_SAMPLES/2) {
					    		Capture.captureState = CaptureState.Idle;
					    		Main.forklift.ungrip();
					    		odo.stopMotors();
					    		// back off to avoid dropping the claw on top of the block
					    		odo.moveCM(LINEDIR.Backward, Util.ROBOT_LENGTH/2, true);
					    		// 180 no scope
					    		search.FOV(Math.PI);
					    		// found lost block
					    		if(Search.isStyrofoamBlock()) Capture.captureState = CaptureState.Grab;
					    		else { 
					    			// give up on finding block, 
					    			Capture.captureState = CaptureState.Idle;
					    			nav.travelTo(search.cardinals[search.currCardinal][0], search.cardinals[search.currCardinal][1]);
					    			Search.searchState = SearchState.Default;
				    			}
				    		}
				    	}
					    try {Thread.sleep(Util.SLEEP_PERIOD);} catch (Exception ex) {}
					  }
				}).start();
				*/
				odo.moveCM(Odometer.LINEDIR.Backward, 3, true); //Back up to avoid bumping into things when spinning
				nav.travelTo(cardinalPoint[0], cardinalPoint[1]);
				if(towerHeight == 0){
					nav.travelTo(towerPosition[0], towerPosition[1]);
					while(Navigation.PathBlocked) {
						Avoider.avoidState = Avoider.AvoidState.Enabled;
						try { Thread.sleep(2*Util.SLEEP_PERIOD); } catch(Exception ex) {}
						while(Main.state == RobotState.Avoiding) {
							try { Thread.sleep(2*Util.SLEEP_PERIOD); } catch(Exception ex) {}
						}
						nav.travelTo(towerPosition[0],towerPosition[1]);
					}
					captureState = CaptureState.Stack;
					odo.moveCM(Odometer.LINEDIR.Backward, Util.CLAW_TO_CENTER, true);

					break;
				}
				double targetHeading = Math.atan2(-odo.getY() + towerPosition[1],-odo.getX() + towerPosition[0]);
				nav.turnTo(targetHeading,true); //Turn to face tower position, stop motors
				odo.setMotorSpeed(Odometer.NAVIGATE_SPEED); //Move forward until the tower is detected.
				odo.forwardMotors();
				while(Main.usSensor.getMedianSample(Util.US_SAMPLES) > Util.TOWER_DISTANCE);
				odo.stopMotors();
				captureState = CaptureState.Stack;
				break;
			case Stack:
				//TODO: Make sure tower is in range
				Main.forklift.liftToTower(towerHeight++);
				Main.forklift.ungrip();
				Main.forklift.liftUp();
				if(Search.searchState != SearchState.Default) Search.searchState = SearchState.AtDropZone; //Pass control back to search
				captureState = CaptureState.Idle;
				break;
			case Idle:
				// iddle state, waiting for avoidance to return
				try{
					Thread.sleep(Util.SLEEP_PERIOD);
				} catch(Exception e) {}
				break;
			default:
				break;
			}
		}
	}
	
	/**
	 * Checks if the position is in field bounds
	 * @param x
	 * @param y
	 * @param width field width (x)
	 * @param height field height (y)
	 * @return
	 */
	public static boolean inBounds(double x,double y, double width, double height) {
		return (x < width) && (y < height) && x > 0 && y > 0;
	}
	
	// sets the current cardinal point at which the robot is bound to
	public static void setContext(double[] cardinalPoint) {
		Capture.cardinalPoint = cardinalPoint;
	}
}
