package chassis;

/*
 * AUTHORS
 * Harley Wiltzer
 * /

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import utilities.Util;

/*
 * START ROBOT WITH FORKLIFT ALL THE WAY UP
 */
/**
 * Forklift abstraction. Use these methods when controlling the forklift.
 * All methods assume robot started with forklift at highest point.
 * @version 3.0
 * @author juliette
 */
public class Forklift {
	private EV3LargeRegulatedMotor forkliftMotor;
	private EV3MediumRegulatedMotor clawMotor;
	private boolean gripped;
	/**
	 * Forklift constructor
	 * @param forkliftMotor motor object for forklift height
	 * @param clawMotor motor object for claw
	 */
	public Forklift(EV3LargeRegulatedMotor forkliftMotor, EV3MediumRegulatedMotor clawMotor) {
		this.forkliftMotor = forkliftMotor;
		this.clawMotor = clawMotor;
		this.gripped = false;
		forkliftMotor.setAcceleration(Util.FORKLIFT_ACCEL);
		clawMotor.setAcceleration(Util.CLAW_ACCEL);
	}
	/**
	 * Move the lift all the way up
	 */
	public void liftUp() { //Move lift all the way up
		forkliftMotor.rotateTo(0);
	}
	/**
	 * Move the lift all the way down (to the ground)
	 */
	public void liftDown() { //Move lift all the way down (to ground)
		int theta = (int)((Util.FORKLIFT_HEIGHT / Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(-theta);
	}
	/**
	 * Move lift to a height based on the current tower's height
	 * @param towerHeight current number of blocks in the tower
	 */
	public void liftToTower(int towerHeight) { //Move lift to height defined by the current tower height
		double height = (towerHeight == 0) ? 0.0 : (double) towerHeight * Util.FOAM_HEIGHT + Util.DROP_SPACE;
		int theta = (int)(((Util.FORKLIFT_HEIGHT - height)/Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(-theta);
	}
	/**
	 * Grip with the claw if not gripping, otherwise ungrip.
	 */
	public void toggleGrip() {
		if(gripped) ungrip();
		else grip();
		gripped = !gripped;
	}
	/**
	 * Move the claw to grip an object
	 */
	public void grip() {
		clawMotor.rotateTo(-Util.GRIP_STRENGTH); //Rotate to an absolute position to apply continous force on block
	}
	/**
	 * Move the claw to release an object
	 */
	public void ungrip() {
		clawMotor.rotateTo(0);
	}
	/**
	 * Get the gripping position of the claw
	 * @return tachometer count for the claw motor in its current position
	 */
	public double getGrip() {
		return clawMotor.getTachoCount();
	}
}
