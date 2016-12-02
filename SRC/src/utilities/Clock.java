package utilities;

/*
 * AUTHORS
 * Juliette Regimbal
 * Harley Wiltzer
 */

import chassis.Main;
import chassis.Main.RobotState;
import utilities.Avoider.AvoidState;
import utilities.Odometer.LINEDIR;

/**
 * Returns the robot to the starting corner before the match ends.
 * Should be started when parameters are set.
 * @version 3.0
 * @author juliette
 *
 */
public class Clock extends Thread{
	public static boolean GAME_OVER = false;
	public static final long FIVE_MINUTES = 1000*60*5;	//game length in milliseconds
	private static final long TIME_THRESHOLD = 1000 * 30;	//time allowed to return to the starting corner
	private Odometer odo;
	private Navigation nav;
	public long startTime;	//public for testing purposes
	
	public Clock(Odometer odo) {
		this.odo = odo;
		this.nav = new Navigation(this.odo);
	}
	
	/**
	 * Clock thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		startTime = System.currentTimeMillis();
		while(true) {
			if(System.currentTimeMillis() - startTime >= FIVE_MINUTES - TIME_THRESHOLD) {
				odo.stopMotors();
				Main.state = Main.RobotState.Returning;	//prevents other threads from acting (except avoidance)
				double[] startingCorner = new double[] { Main.startingCornerCoord[0], Main.startingCornerCoord[1] };
				//navigate to first cardinal point
				while(Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, startingCorner) > Util.CM_TOLERANCE) {
					nav.travelTo(startingCorner[0], startingCorner[1]);
					
					if(Navigation.PathBlocked) {
						odo.moveCM(Odometer.LINEDIR.Backward,Util.BACKUP_DISTANCE,true);
						Avoider.avoidState = AvoidState.Enabled;
						// wait for avoider to finish
						//avoid race condition
						try {Thread.sleep(2*Util.SLEEP_PERIOD);} catch (Exception ex) {}
						while(Main.state == RobotState.Avoiding) {
							try{Thread.sleep(Util.SLEEP_PERIOD);}catch(Exception ex) {}
						}
					}
				}
				
				odo.moveCM(LINEDIR.Forward, 5, true); //move forward to fully be in the starting corner
				
				odo.stopMotors();
				Main.state = Main.RobotState.Finished;
				
				Clock.GAME_OVER = true;
				break; //end thread
			}
			else try { Thread.sleep(500); } catch(Exception ex) {}
		}
	}
}
