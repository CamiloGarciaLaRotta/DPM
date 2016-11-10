package utilities;

import lejos.hardware.Button;

/**
 * Thread that terminates program when escape pressed
 * @version 0.2
 * @author juliette
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
