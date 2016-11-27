package utilities;

public class Clock extends Thread{
	public static boolean GAME_OVER = false;
	private static final long FIVE_MINUTES = 1000*60*5;
	private static final long TIME_THRESHOLD = 1000 * 30;
	private Odometer odo;
	
	public Clock(Odometer odo) {
		this.odo = odo;
	}
	
	public void run() {
		long startTime = System.currentTimeMillis();
		while(true) {
			if(System.currentTimeMillis() - startTime >= FIVE_MINUTES - TIME_THRESHOLD) {
				//TODO Stop flow, travelTo starting corner
				Clock.GAME_OVER = true;
			}
			else try { Thread.sleep(500); } catch(Exception ex) {}
		}
	}
}
