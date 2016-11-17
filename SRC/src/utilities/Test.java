package utilities;

import chassis.ColorSensor;
import chassis.LCDInfo;
import chassis.Main;
import chassis.USSensor;
import chassis.Main.RobotState;
import wifi.StartCorner;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import utilities.Avoider.AvoidState;

/**
 * Testing methods for the robot
 * @version 3.0
 * 
 */
public class Test {
	/**
	 * Simple localization routine
	 * @param odo Odometer
	 */
	public static void LocalizationTest(Odometer odo) {
		USLocalizer localizer = new USLocalizer(odo, Main.usSensor, Util.US_TO_CENTER);
		localizer.doLocalization();
	}

	/**
	 * Move robot in square pattern (clockwise) of arbitrary size, and arbitrary amount of laps
	 * @param odo Odometer
	 * @param laps Amount of laps for robot to travel
	 * @param length Length of square
	 */
	public static void SquareTest(Odometer odo, int laps, double length) {
		double[] waypoints = new double[laps * 8];
		
		//OdometryCorrection correct = new OdometryCorrection(odo);
		//correct.start();
		
		for(int c = 0; c < laps; c++) {
			waypoints[8*c] = 0.0;
			waypoints[8*c+1] = 0.0;
			
			waypoints[8*c+2] = length;
			waypoints[8*c+3] = 0.0;
			
			waypoints[8*c+4] = length;
			waypoints[8*c+5] = length;
			
			waypoints[8*c+6] = 0.0;
			waypoints[8*c+7] = length;
			if(Button.readButtons() == Button.ID_ESCAPE) return;
		}
		
		Navigation nav = new Navigation(odo);
		
		for(int c = 0; c < waypoints.length/2; c++) {
			nav.travelTo(waypoints[2*c], waypoints[2*c + 1]);
			if(Button.readButtons() == Button.ID_ESCAPE) return;
		}
		
		nav.travelTo(0, 0);
		nav.turnTo(0, true);
		
	}
	
	/**
	 * Move robot forward and backward, turn 90 degrees and repeat
	 * @param odo Odometer
	 * @param distance Distance to move forward and backward
	 */
	public static void StraightLineTest(Odometer odo, double distance) {
		Navigation nav = new Navigation(odo);
		for(int c = 0; c < 4; c++) {
			odo.moveCM(Odometer.LINEDIR.Forward, distance, true);
			
			if(Button.waitForAnyPress() == Button.ID_ESCAPE) break;
			
			odo.moveCM(Odometer.LINEDIR.Backward, distance, true);
			
			Button.waitForAnyPress();
			
			nav.turnBy(Math.PI/2);
		}
	}
	
	/**
	 * Move robot to arbitrary waypoints
	 * @param odo Odometer
	 * @param points Waypoints
	 * @param stop Stop robot at each point to measure error
	 */
	public static void NavigationTest(Odometer odo, double[][] points, boolean stop) {
		Navigation nav = new Navigation(odo);

		for(int i = 0; i < points.length; i++) {
			nav.travelTo(points[i][0], points[i][1]);
			if(stop){
				Sound.beep();
				Button.waitForAnyPress();
			}
		}
	}
	
	public static void TrackMeasureTest(Odometer odo, int rotations) {
		Navigation nav = new Navigation(odo);
		for(int i = 0; i < rotations; i++) {
			nav.turnBy(2*Math.PI);
		}
	}
	
	/**
	 * Measure RGB unit vectors
	 * @param color - ColorSensor object
	 */
	public static void RGBUnitVectorTest(ColorSensor color) {
		float [] rgbRaw = new float[3];
		while(Button.readButtons() != Button.ID_ESCAPE) {
			rgbRaw = color.getColor();
			double magnitude = Math.sqrt(rgbRaw[0]*rgbRaw[0] + rgbRaw[1]*rgbRaw[1] + rgbRaw[2]*rgbRaw[2]);
			rgbRaw[0] /= magnitude;
			rgbRaw[1] /= magnitude;
			rgbRaw[2] /= magnitude;
			LCDInfo.displayMessage(rgbRaw[0] + " " + rgbRaw[1] + " " + rgbRaw[2]);
		}
	}
	
	/**
	 * Tests object differentiation in Search thread
	 * @see Search
	 * @param odometer - Odometer Object
	 * @param colorSensor - ColorSensor Object
	 * @param usSensor - USSensor Object
	 * @param GREEN - coordinates of scoring zone
	 */
	public static void ObjectDifferentiationTest(Odometer odometer, ColorSensor colorSensor, USSensor usSensor, double[][] GREEN) {
		Search search = new Search(odometer, colorSensor, usSensor, GREEN);
		Main.state = Main.RobotState.Search;
		search.run();
		while(Main.state == Main.RobotState.Search && Button.readButtons() != Button.ID_ESCAPE);
		search.interrupt();
	}
	
	public static void ForkliftTest() {
		int option;
		do {
			option = Button.waitForAnyPress();
			switch(option) {
			case Button.ID_UP:
				Main.forklift.liftUp();
				break;
			case Button.ID_DOWN:
				Main.forklift.liftDown();
				break;
			case Button.ID_ENTER:
				Main.forklift.toggleGrip();
				break;
			default:
				break;
			}
		}while(option != Button.ID_ESCAPE);
	}
	
	/*public static void AvoidanceTest(Odometer odometer) {
		//set up
		Navigation nav = new Navigation(odometer);
		Main.lcd.setLine1("Put robot at origin");
		Main.lcd.setLine2("Place obstacle directly ahead");
		Button.waitForAnyPress();
		odometer.setPosition(new double[] {0, 0, Math.PI/2}, new boolean[] {true, true, true});
		
		//obstacle avoidance
		nav.travelTo(0, 60);	//should cross paths with block
		
		Main.lcd.setLine1("Will now treat next zone as RED");
		Main.lcd.setLine2("Press a button");
		
		//Red zone
		Main.RED = new double[][] {{odometer.getX() + 20, odometer.getY() - 20}, {odometer.getX() + 50, odometer.getY() + 20}};
		nav.travelTo(70, 60);
		
		Main.lcd.setLine1("Will attempt to go to corner");
		Main.lcd.setLine2("Should actually go to green zone");
		
		//Corner
		StartCorner corner = StartCorner.BOTTOM_LEFT;
		nav.travelTo(corner.getX(), corner.getY());
		
		Main.lcd.setLine1("Record results for all tests then exit");
		
	}*/
	
	public static void AvoidanceTest(Odometer odo) {
		Navigation nav = new Navigation(odo);
		
		Button.waitForAnyPress();
		odo.setPosition(new double[] {0, 0, Math.PI/2}, new boolean[] {true, true, true});
		
		double[] endPos = {0, 150};
		while(Odometer.euclideanDistance(endPos, new double[] {odo.getX(), odo.getY()}) > Util.CM_TOLERANCE) {
			nav.travelTo(endPos[0], endPos[1]);
			if(Navigation.PathBlocked) {
				Avoider.avoidState = Avoider.AvoidState.Enabled;
				
				try {Thread.sleep(2*Util.SLEEP_PERIOD);} catch (Exception ex) {}
				while(Main.state == RobotState.Avoiding) {
					try{Thread.sleep(Util.SLEEP_PERIOD);}catch(Exception ex) {}
				}
				Avoider.avoidState = AvoidState.Disabled;
			}
		}
		Main.lcd.setLine1("Done");
	}
	
}
