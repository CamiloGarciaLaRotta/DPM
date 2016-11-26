package utilities;

import chassis.Main;
import chassis.Main.RobotState;
import utilities.Avoider.AvoidState;
import utilities.Odometer.LINEDIR;

public class Clock extends Thread{
	public static boolean GAME_OVER = false;
	private static final long FIVE_MINUTES = 1000*60*5;	//game length in milliseconds
	private static final long TIME_THRESHOLD = 1000 * 30;
	private Odometer odo;
	private Navigation nav;
	
	public Clock(Odometer odo) {
		this.odo = odo;
		this.nav = new Navigation(this.odo);
	}
	
	public void run() {
		long startTime = System.currentTimeMillis();
		while(true) {
			if(System.currentTimeMillis() - startTime >= FIVE_MINUTES - TIME_THRESHOLD) {
				//TODO Stop flow, travelTo starting corner
				odo.stopMotors();
				Main.state = Main.RobotState.Returning;	//prevents other threads from acting (except avoidance)
				double[] startingCorner = new double[] { Main.startingCornerCoord[0], Main.startingCornerCoord[1] };
				//navigate to first cardinal point
				while(Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, startingCorner) < Util.CM_TOLERANCE) {
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
