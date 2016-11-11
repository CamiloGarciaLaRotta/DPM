package chassis;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Util;

/**
 * START ROBOT WITH FORKLIFT ALL THE WAY UP
 */

public class Forklift {
	private EV3LargeRegulatedMotor forkliftMotor;
	private EV3LargeRegulatedMotor clawMotor;

	public Forklift(EV3LargeRegulatedMotor forkliftMotor, EV3LargeRegulatedMotor clawMotor) {
		this.forkliftMotor = forkliftMotor;
		this.clawMotor = clawMotor;
	}

	public void liftUp() { //Move lift all the way up
		forkliftMotor.rotateTo(0);
	}

	public void liftDown() { //Move lift all the way down (to ground)
		int theta = (int)((Util.FORKLIFT_HEIGHT / Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(theta);
	}

	public void liftToTower(int towerHeight) { //Move lift to height defined by the current tower height
		double height = (double) towerHeight * Util.FOAM_HEIGHT;
		int theta = (int)(((Util.FORKLIFT_HEIGHT - towerHeight * Util.FOAM_HEIGHT)/Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(theta);
	}

	public void grip() {
		clawMotor.rotateTo(Util.GRIP_STRENGTH);
	}

	public void ungrip() {
		clawMotor.rotateTo(0);
	}
}
