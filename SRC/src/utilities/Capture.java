package utilities;

import chassis.Main;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Search.SearchState;

public class Capture extends Thread {
	private Odometer odo;
	private Navigation nav;
	
	private int towerHeight;
	
	public enum CaptureState {Disabled, Grab, Return, Stack};
	public enum ForkliftPosition {Ground, Up, Tower, Block};
	public static CaptureState captureState = CaptureState.Disabled;
	
	private ForkliftPosition liftPos;
	
	private double[][] green;
	
	/**
	 * Capture Thread Constructor
	 * @param odometer - Odometer Object
	 * @param leftArm - motor for left arm
	 * @param rightArm - motor for right arm
	 */
	public Capture(Odometer odometer, EV3LargeRegulatedMotor leftArm, EV3LargeRegulatedMotor rightArm, double[][] green) {
		this.odo = odometer;
		this.nav = new Navigation(this.odo);
		this.green = green;
		this.towerHeight = 0;
		this.liftPos = ForkliftPosition.Up;
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
				moveForklift(ForkliftPosition.Block);
				//TODO: Close claw
				moveForklift(ForkliftPosition.Up);
				captureState = CaptureState.Return;
				break;
			case Return:
				odo.moveCM(Odometer.LINEDIR.Backward, 3, true);
				nav.turnBy(Math.PI);
				odo.setMotorSpeed(Odometer.NAVIGATE_SPEED);
				odo.forwardMotors();
				//TODO: Update this with position of tower
				while(Odometer.euclideanDistance(new double[] {odo.getX(),odo.getY()}, new double[] {green[0][0],green[0][1]}) > Util.ZONE_THRESHOLD);
				odo.stopMotors();
				captureState = CaptureState.Stack;
				break;
			case Stack:
				//TODO: Make sure tower is in range
				moveForklift(ForkliftPosition.Tower);
				odo.moveCM(Odometer.LINEDIR.Backward, 2, true);
				captureState = CaptureState.Disabled;
				break;
			default:
				break;
			}
		}
	}
	
	private void moveForklift(ForkliftPosition pos) {
		double theta = 0;
		if(pos == this.liftPos) return;
		//TODO: Figure out relationship between motor angle and forklift position
		switch(pos) {
		case Ground:
			//TODO: theta --> full length of forklift
			break;
		case Up:
			theta = 0;
			break;
		case Tower:
			//TODO: theta --> towerHeight * Util.BLOCK_HEIGHT
			break;
		case Block:
			//TODO: theta --> full length of forklift - Util.BLOCK_HEIGHT
			break;
		}
		//TODO: Motor.rotate(theta)
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
