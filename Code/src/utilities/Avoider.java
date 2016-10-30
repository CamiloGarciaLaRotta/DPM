package utilities;

import chassis.Lab5;

public class Avoider extends Thread{
	
	private Odometer odo;
	private Navigation nav;
	
	private static final int AVOID_SPEED	= 60;
	
	public Avoider(Odometer odo) {
		this.odo = odo;
	}

	@Override
	public void run() {
		while(Lab5.state != Lab5.RobotState.k_Avoiding) {
			try { Thread.sleep(300); } catch(Exception ex) {ex.printStackTrace();}
		}
		
	}
	
}
