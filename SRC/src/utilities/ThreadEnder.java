package utilities;

import lejos.hardware.Button;

public class ThreadEnder extends Thread{

	public void run() {
		while(true)
			if(Button.waitForAnyPress() == Button.ID_ESCAPE) System.exit(1);
	}
	
}
