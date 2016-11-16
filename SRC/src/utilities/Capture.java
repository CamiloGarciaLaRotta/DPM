package utilities;

import chassis.Main;
import chassis.Main.RobotState;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	public Capture(Odometer odometer, double[][] GREEN) {
		this.odo = odometer;
		this.nav = new Navigation(this.odo);
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
				Main.forklift.grip();
				Main.forklift.liftUp();
				captureState = CaptureState.Return;
				break;
			case Return:
				odo.moveCM(Odometer.LINEDIR.Backward, 3, true); //Back up to avoid bumping into things when spinning
				nav.travelTo(cardinalPoint[0], cardinalPoint[1]);
				if(towerHeight == 0){
					nav.travelTo(towerPosition[0], towerPosition[1]);
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
				Search.searchState = SearchState.AtDropZone; //Pass control back to search
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
