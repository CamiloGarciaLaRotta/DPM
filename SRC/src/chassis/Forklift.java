package chassis;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import utilities.Util;

/**
 * START ROBOT WITH FORKLIFT ALL THE WAY UP
 */

public class Forklift {
	private EV3LargeRegulatedMotor forkliftMotor;
	private EV3MediumRegulatedMotor clawMotor;
	private boolean gripped;

	public Forklift(EV3LargeRegulatedMotor forkliftMotor, EV3MediumRegulatedMotor clawMotor) {
		this.forkliftMotor = forkliftMotor;
		this.clawMotor = clawMotor;
		this.gripped = false;
		forkliftMotor.setAcceleration(Util.FORKLIFT_ACCEL);
		clawMotor.setAcceleration(Util.CLAW_ACCEL);
	}

	public void liftUp() { //Move lift all the way up
		forkliftMotor.rotateTo(0);
	}

	public void liftDown() { //Move lift all the way down (to ground)
		int theta = (int)((Util.FORKLIFT_HEIGHT / Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(-theta);
	}

	public void liftToTower(int towerHeight) { //Move lift to height defined by the current tower height
		double height = (towerHeight == 0) ? 0.0 : (double) towerHeight * Util.FOAM_HEIGHT + Util.DROP_SPACE;
		int theta = (int)((height/Util.FORKLIFT_ROPE_RADIUS) * 180.0 / Math.PI);
		forkliftMotor.rotateTo(theta);
	}
	
	public void toggleGrip() {
		if(gripped) ungrip();
		else grip();
		gripped = !gripped;
	}

	public void grip() {
		clawMotor.rotateTo(-Util.GRIP_STRENGTH);
	}

	public void ungrip() {
		clawMotor.rotateTo(0);
	}
}
