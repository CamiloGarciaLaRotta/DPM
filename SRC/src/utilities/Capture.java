package utilities;

import chassis.Main;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Search.SearchState;

public class Capture extends Thread {
	
	// instances
	private Odometer odo;
	private Navigation nav;
	
	// coordinates
	private double[][] GREEN;
	private double[] towerPosition;
	
	// states
	public enum CaptureState {Disabled, Grab, Return, Stack};
	public enum ForkliftPosition {Ground, Up, Tower};
	public static CaptureState captureState = CaptureState.Disabled;

	private EV3LargeRegulatedMotor forkliftMotor;
	private EV3LargeRegulatedMotor clawMotor;
	private ForkliftPosition liftPos;
	
	private int towerHeight;
	
	//TODO TODO TODO TODO
	// -  when retrieving: return with block to cardinal point
	//					   turn to face center of GREEN
	//					   advance till tower
	//					   drop block
	//					   set searchState.AtDropZones
	
	/**
	 * Capture Thread Constructor
	 * @param odometer - Odometer Object
	 * @param leftArm - motor for left arm
	 * @param rightArm - motor for right arm
	 */
	public Capture(Odometer odometer, EV3LargeRegulatedMotor forkliftMotor, EV3LargeRegulatedMotor clawMotor, double[][] GREEN) {
		this.odo = odometer;
		this.nav = new Navigation(this.odo);
		this.GREEN = GREEN;
		this.towerHeight = 0;
		this.liftPos = ForkliftPosition.Up;
		this.forkliftMotor = forkliftMotor;
		this.clawMotor = clawMotor;
		this.towerPosition = new double[]{(GREEN[0][0] + GREEN[1][0])/2,(GREEN[0][1] + GREEN[1][1])/2};
	}
	
	/**
	 * start Capture thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(true) {
			switch(captureState) {
			case Disabled:
				try {
					Thread.sleep(500);
				}catch(Exception ex) { ex.printStackTrace(); }
				break;
			case Grab:
				//TODO: Make sure block is in range
				moveForklift(ForkliftPosition.Ground); //Descend forklift
				grip();
				moveForklift(ForkliftPosition.Up); //Raise forklift to top to avoid colliding into stuff
				captureState = CaptureState.Return;
				break;
			case Return:
				odo.moveCM(Odometer.LINEDIR.Backward, 3, true); //Back up to avoid bumping into things when spinning
				double targetHeading = Math.atan2(odo.getY() - towerPosition[1],odo.getX() - towerPosition[0]);
				nav.turnTo(targetHeading,true); //Turn to face tower position, stop motors
				odo.setMotorSpeed(Odometer.NAVIGATE_SPEED); //Move forward until the tower is detected.
				odo.forwardMotors();
				while(Main.usSensor.getMedianSample(Util.US_SAMPLES) > Util.TOWER_DISTANCE);
				odo.stopMotors();
				captureState = CaptureState.Stack;
				break;
			case Stack:
				//TODO: Make sure tower is in range
				moveForklift(ForkliftPosition.Tower); //Descend forklift to height of current tower
				ungrip();
				moveForklift(ForkliftPosition.Up);
				odo.moveCM(Odometer.LINEDIR.Backward, 5, true); //Back up to avoid bumping into tower
				Search.searchState = SearchState.AtDropZone; //Pass control back to search
				captureState = CaptureState.Disabled;
				break;
			default:
				break;
			}
		}
	}
	
	private void moveForklift(ForkliftPosition pos) {
		//Moves forklift to either the ground position, up position, or to the height of the tower.
		//Formula for converting height to motor tacho position is convenient in radians.
		//However, EV3LargeRegulatedMotor's rotate function TAKE DEGREES. Be sure to switch.
		double theta = 0;
		if(pos == this.liftPos) return;
		this.liftPos = pos;
		switch(pos) {
		case Ground:
			theta = Util.FORKLIFT_HEIGHT / Util.FORKLIFT_ROPE_RADIUS;
			break;
		case Up:
			theta = 0;
			break;
		case Tower:
			theta = (Util.FORKLIFT_HEIGHT - towerHeight * Util.FOAM_HEIGHT) / Util.FORKLIFT_ROPE_RADIUS;
			break;
		default:
			break;
		}
		forkliftMotor.rotateTo((int)(theta * 180 / Math.PI)); //Converting theta to degrees and rotating.
	}

	private void grip() {
		clawMotor.rotateTo(Util.GRIP_STRENGTH);
	}

	private void ungrip() {
		clawMotor.rotateTo(0);
	}
	
	/**
	 * Checks if the position is in field bounds
	 * @param x
	 * @param y
	 * @param width - field width (x)
	 * @param height - field height (y)
	 * @return
	 */
	public static boolean inBounds(double x,double y, double width, double height) {
		return (x < width) && (y < height) && x > 0 && y > 0;
	}
}
