package utilities;

import chassis.Main;

/**
 * @version 0.1
 * @author juliette
 * Checks if the robot must avoid an object in its path and then does it
 */
public class Avoider extends Thread{
	
	private Odometer odo;
	private Navigation nav;
	
	private static final int AVOID_SPEED = 60;
	
	//TODO complete and test
	
	public Avoider(Odometer odo) {
		this.odo = odo;
	}

	@Override
	public void run() {
		while(Main.state != Main.RobotState.Avoiding) {
			try { Thread.sleep(300); } catch(Exception ex) {ex.printStackTrace();}
		}
		
	}
	
}
