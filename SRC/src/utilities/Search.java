package utilities;

import chassis.ColorSensor;
import chassis.Main;
import chassis.USSensor;
import lejos.hardware.Sound;
import utilities.Odometer.LINEDIR;

public class Search extends Thread {
	private static final int US_SAMPLES = 10;
	private Odometer odo;
	private USSensor usSensor;
	private ColorSensor colorSensor;
	private final float FIELD_BOUNDS = 65; //cm
	private float lastDistanceDetected;
	private final float DISTANCE_THRESHOLD = 25; //cm
	private final double fieldToSearch = Math.PI/2;
	private final float[][] corners = new float[][] {{0,0},{0,60.96f},{60.96f,60.96f},{60.96f,0.0f}};
	private int corner;
	private int dir;
	private final static float BLOCK_DISTANCE = 4.0f; //distance to detect block type in cm
	public static double[] blockLocation;
	public static double[] obstacleLocation;
	private final int APPROACH_SPIN_SPEED = 75;
	private Navigation nav;
	
	private boolean method1;
		
	public static final float[] STYROFOAM_COLOR = new float[] {0.0f,1.0f,1.0f};
	
	public Search(Odometer odo, ColorSensor colorSensor, USSensor usSensor) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.corner = 0;
		this.dir = 1;
		nav = new Navigation(odo);
		method1 = true;
	}
	@Override
	public void run() {
		while(Main.state != Main.RobotState.Search) {
			//wait for localization to finish
			try {
				Thread.sleep(300);
			} catch (Exception e) {}
		}
		
		boolean found = false;
		
		if(!method1) {
			int dir = 1;
			isStyrofoamBlock(); //Initialize rgb mode
			
			while(!found) {
				odo.setMotorSpeed(APPROACH_SPIN_SPEED);
				odo.forwardMotors();
				while(usSensor.getMedianSample(US_SAMPLES) > BLOCK_DISTANCE && (odo.getX() < 58 && odo.getY() < 58));
				odo.stopMotors();
				
				if(odo.getX() >= 55 || odo.getY() >= 55) {
					odo.moveCM(LINEDIR.Backward, 13, true);
					nav.turnBy(Math.pow(-1, dir) * Math.PI/2);
					dir++;
					Sound.beepSequence();
				}
				else if(usSensor.getMedianSample(US_SAMPLES) <= BLOCK_DISTANCE && isStyrofoamBlock()) {
					found = true;
					Sound.beep();
					odo.moveCM(LINEDIR.Backward, 13, true);
					Main.state = Main.RobotState.Capture;
				}
				else {
					Sound.twoBeeps();
					odo.moveCM(LINEDIR.Backward, 13, true);
					nav.turnBy(Math.PI/2);
				}
			}
		}

		else { //Temporary, in case we want to switch back to this method
			//Comb through track, check for detection
			//sweep track for obstacles from position (0,0) starting at 0-radians
			boolean blockFound=false;
			
			isStyrofoamBlock(); //Initialize rgb mode
			
			while(!blockFound) {
				lastDistanceDetected = FIELD_BOUNDS;
				boolean objectFound = false;
				odo.setMotorSpeeds(Odometer.ROTATE_SPEED, Odometer.ROTATE_SPEED);
				
				//If last travelTo was interrupted by an obstacle, go back to previous corner and switch directions
				if(Navigation.PathBlocked) {
					dir = -dir;
					corner += dir;
					corner %= 4;
					nav.travelTo(corners[corner][0], corners[corner][1]);		
					int nextCorner = (corner + dir < 0) ? corner + dir + 4 : (corner + dir) % 4;
					nav.turnTo(Math.atan2(corners[nextCorner][1], corners[nextCorner][0]), true);
				}
								
				double targetAngle = odo.getTheta() - dir*fieldToSearch;
				if(targetAngle > 2*Math.PI) targetAngle -= 2*Math.PI;
				else if (targetAngle < 0) targetAngle += 2*Math.PI;
				odo.setMotorSpeed(70);
				odo.spin(dir == 1 ? Odometer.TURNDIR.CW : Odometer.TURNDIR.CCW);
				try{
					Thread.sleep(500);
				} catch(Exception e) {} //we're living dangerously
				while(!(objectFound = isObjectDetected()) && Math.abs(Navigation.minimalAngle(targetAngle, odo.getTheta())) > Math.PI/60);	//check if there is an object at current heading or if area has been scanned
				if(objectFound) {	//go to object, check if it is a styrofoam block
					Sound.beep();
					odo.stopMotors();
										
					double distance = usSensor.getMedianSample(US_SAMPLES); //Get distance to detected object
					double heading = odo.getTheta();
					
					odo.setMotorSpeed(70);
					odo.forwardMotors();
					int notDir = dir;
					while(usSensor.getMedianSample(US_SAMPLES) > 6) {
						while(usSensor.getMedianSample(US_SAMPLES) > distance) {
							if(Math.abs(odo.getTheta() - heading) > Math.PI/4) {
								odo.moveCM(Odometer.LINEDIR.Forward, 0.5, true);
								notDir = -notDir;
							}
							odo.setMotorSpeed(APPROACH_SPIN_SPEED);
							if(notDir == 1) odo.spin(Odometer.TURNDIR.CW);
							else odo.spin(Odometer.TURNDIR.CCW);
						}
						odo.forwardMotors();
					}
					odo.stopMotors();
					Sound.beep();
					odo.stopMotors();
					odo.setMotorSpeeds(60, 60);
					odo.forwardMotors();
					while(usSensor.getMedianSample(US_SAMPLES) > BLOCK_DISTANCE); //wait until close enough to determine if it's a styrofoam block
					odo.setMotorSpeeds(0, 0);
					odo.forwardMotors();
					if(isStyrofoamBlock()) {	//begin capture
						blockFound = true;
						Sound.beepSequenceUp();
					} else {
					}
					odo.moveCM(LINEDIR.Backward, 13, true); //Move backward, to avoid spinning into obstacle
					if(!blockFound) { //If the obstacle wasn't a styrofoam block, go to next corner
						nav.turnBy(dir*Math.PI/2);
					}
				} else {	//go to next corner
					nav.turnBy(dir*Math.PI/2);
					odo.setMotorSpeed(APPROACH_SPIN_SPEED);
					odo.forwardMotors();
					while(usSensor.getMedianSample(US_SAMPLES) < 10 && !Capture.inBounds(odo.getX(), odo.getY(), Util.FIELD_BOUNDARY, Util.FIELD_BOUNDARY));
					odo.stopMotors();
					odo.moveCM(Odometer.LINEDIR.Backward, 5, true);
				}
			}
			
			//Styrofoam block found - begin capture
			Sound.twoBeeps();
			Main.state = Main.RobotState.Capture; //If blockFound, switch to Capture state
		} /*else { //PART 1
			while(true) {
				chassis.LCDInfo.getLCD().clear();
				if(usSensor.getMedianSample(US_SAMPLES) <= BLOCK_DISTANCE) {
					//Sound.beep();
					Util.lcd.setLine1("Object detected");
					if(isStyrofoamBlock()) {
						Util.lcd.setLine2("Block");
						Sound.beep();
					} else {
						Util.lcd.setLine2("Not a block");
						Sound.twoBeeps();
					}
				}
				try {
					Thread.sleep(500);
				} catch(Exception e) { }
			}
		}*/
	}
	
	private boolean isStyrofoamBlock() {
		return (colorSensor.getColor()[0] < colorSensor.getColor()[1]);
		//return (colorDistance(colorSensor.getColor(),STYROFOAM_COLOR) < 1.0);
	}
	
	private boolean isObjectDetected() {
		float currentDistance = usSensor.getMedianSample(US_SAMPLES);
		boolean isObject = (lastDistanceDetected - currentDistance > DISTANCE_THRESHOLD) && (currentDistance <= 45);
		lastDistanceDetected = currentDistance;
		return isObject;
	}
	
	public static float colorDistance(float[] a, float[] b){
		return (float)Math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
	}
}
