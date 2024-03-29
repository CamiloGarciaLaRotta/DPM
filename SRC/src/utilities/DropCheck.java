package utilities;

/*
 * AUTHORS
 * Camilo Garcia La Rotta
 * Juliette Regimbal
 */

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import chassis.Main;
import chassis.Main.RobotState;
import utilities.Capture.CaptureState;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * Runs during capture to make sure the block was not dropped.
 * Tries to retrieve the block if it is dropped.
 * @version 3.0
 * @author juliette
 */
public class DropCheck extends Thread{

	private Odometer odo;
	private Navigation nav;
	private Search search;
	public boolean dropped;
	public Lock interrupt;
	private Object dropMutex;
	
	/**
	 * DropCheck Constructor
	 * @param odo Odometer object
	 * @param search Search thread (to access and set states)
	 */
	public DropCheck(Odometer odo, Search search){
		this.odo = odo;
		this.search = search;
		this.interrupt = new ReentrantLock();
		this.nav = new Navigation(this.odo);
		this.dropMutex = new Object();
	}
	/**
	 * {@inheritDoc}
	 */
	public void run() {
		 // only active during the return state
		setDropped(false);
	    while(Capture.captureState == CaptureState.Return && !this.isInterrupted()){
	    	//If claw tacho count is too close to -180 degrees, block was dropped
	    	if (Main.forklift.getGrip() < Util.GRIP_THRESHOLD) {
	    		interrupt.lock();
	    		//Main.state = RobotState.Disabled;	//prevents other threads from taking control
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
	    			nav.travelTo(Search.cardinals[search.currCardinal][0], Search.cardinals[search.currCardinal][1]);
	    			Search.searchState = SearchState.Default;
	    			Main.state = RobotState.Search;
    			}
	    		interrupt.unlock();
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
