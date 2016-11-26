package utilities;

import chassis.Main;
import lejos.hardware.Button;
import utilities.Capture.CaptureState;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * @version 3.0
 * @author juliette
 * Thread to verify that the robot hasn't dropped the block it's holding.
 */
public class DropCheck extends Thread{

	private Odometer odo;
	private Navigation nav;
	private Search search;
	public boolean dropped;
	
	public DropCheck(Odometer odo, Search search){
		this.odo = odo;
		this.nav = new Navigation(this.odo);
	}
	/**
	 * {@inheritDoc}
	 */
	public void run() {
		while(true){
			  // only active during the return state
			dropped = false;
		    while(Capture.captureState == CaptureState.Return){
		    	//If claw tacho count is too close to -180 degrees, block was dropped
		    	if (Main.forklift.getGrip() < Util.GRIP_THRESHOLD) {
		    		this.dropped = true;
		    		Capture.captureState = CaptureState.Idle;
		    		Main.forklift.ungrip();
		    		odo.stopMotors();
		    		// back off to avoid dropping the claw on top of the block
		    		odo.moveCM(LINEDIR.Backward, Util.ROBOT_WIDTH/2, true);
		    		// 180 no scope
		    		search.FOV(Math.PI);
		    		// found lost block
		    		if(Search.isStyrofoamBlock()) Capture.captureState = CaptureState.Grab;
		    		else { 
		    			// give up on finding block, 
		    			Capture.captureState = CaptureState.Idle;
		    			nav.travelTo(search.cardinals[search.currCardinal][0], search.cardinals[search.currCardinal][1]);
		    			Search.searchState = SearchState.Default;
	    			}
	    		}
	    	}
		    try {Thread.sleep(Util.SLEEP_PERIOD);} catch (Exception ex) {}
		  }
	}
}