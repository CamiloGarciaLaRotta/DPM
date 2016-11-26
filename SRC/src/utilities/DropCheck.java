package utilities;

import chassis.Main;
import chassis.Main.RobotState;
import utilities.Capture.CaptureState;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * @version 3.0
 * @author juliette
 * Runs during capture to make sure the block was not dropped.
 * Tries to retrieve the block if it is dropped.
 */
public class DropCheck extends Thread{

	private Odometer odo;
	private Navigation nav;
	private Search search;
	public boolean dropped;
	private Object dropMutex;
	
	/**
	 * DropCheck Constructor
	 * @param odo Odometer object
	 * @param search Search thread (to access and set states)
	 */
	public DropCheck(Odometer odo, Search search){
		this.odo = odo;
		this.nav = new Navigation(this.odo);
	}
	/**
	 * {@inheritDoc}
	 */
	public void run() {
		 // only active during the return state
		setDropped(false);
	    while(Capture.captureState == CaptureState.Return){
	    	//If claw tacho count is too close to -180 degrees, block was dropped
	    	if (Main.forklift.getGrip() < Util.GRIP_THRESHOLD) {
	    		Main.state = RobotState.Disabled;	//prevents other threads from taking control
	    		setDropped(true);
	    		Capture.captureState = CaptureState.Idle;
	    		Main.forklift.ungrip();
	    		odo.stopMotors();
	    		// back off to avoid dropping the claw on top of the block
	    		odo.moveCM(LINEDIR.Backward, Util.ROBOT_WIDTH/2, true);
	    		// 180 no scope
	    		search.FOV(Math.PI);
	    		// found lost block
	    		if(Search.isStyrofoamBlock()) {
	    			Capture.captureState = CaptureState.Grab;
	    			Main.state = RobotState.Capture;
	    		}
	    		else { 
	    			// give up on finding block, 
	    			Capture.captureState = CaptureState.Idle;
	    			nav.travelTo(search.cardinals[search.currCardinal][0], search.cardinals[search.currCardinal][1]);
	    			Search.searchState = SearchState.Default;
	    			Main.state = RobotState.Search;
	    			return;
    			}
    		}
    	}
	}
	
	/**
	 * If the robot has dropped the block.
	 * Uses mutual exclusion since Capture and DropCheck will both be polling dropped.
	 * @return if the block is dropped
	 */
	public boolean getDropped() {
		synchronized(dropMutex) {
			return dropped;
		}
	}
	
	/**
	 * Sets the drop state of the block. Uses mutual exclusion.
	 * @param dropped if the block should be set to dropped
	 */
	private void setDropped(boolean dropped) {
		synchronized(dropMutex) {
			this.dropped = dropped;
		}
	}
}