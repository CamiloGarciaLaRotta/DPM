package utilities;

/*
 * AUTHORS
 * Harley Wiltzer
 */

import lejos.hardware.Button;

/**
 * Thread that terminates program when escape pressed
 * @version 3.0
 * 
 */
public class ThreadEnder extends Thread{

	/**
	 * {@inheritDoc}
	 */
	public void run() {
		while(true)
			if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(1);
	}
	
}
