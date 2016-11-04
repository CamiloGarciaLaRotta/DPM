package utilities;

import chassis.Main;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * @version 0.1
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
		
		OdometryCorrection correct = new OdometryCorrection(odo);
		correct.start();
		
		for(int c = 0; c < laps; c++) {
			waypoints[4*c] = 0.0;
			waypoints[4*c+1] = 0.0;
			
			waypoints[4*c+2] = Util.SQUARE_LENGTH;
			waypoints[4*c+3] = 0.0;
			
			waypoints[4*c+4] = Util.SQUARE_LENGTH;
			waypoints[4*c+5] = Util.SQUARE_LENGTH;
			
			waypoints[4*c+6] = 0.0;
			waypoints[4*c+7] = Util.SQUARE_LENGTH;
		}
		
		Navigation nav = new Navigation(odo);
		
		for(int c = 0; c < waypoints.length; c++) {
			nav.travelTo(waypoints[2*c], waypoints[2*c + 1]);
		}
		
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
			
			Button.waitForAnyPress();
			
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
	public static void NavigationTest(Odometer odo, int[][] points, boolean stop) {
		Navigation nav = new Navigation(odo);

		for(int i = 0; i < points.length; i++) {
			nav.travelTo(points[i][0], points[i][1]);
			if(stop){
				Sound.beep();
				Button.waitForAnyPress();
			}
		}
	}
	
}
