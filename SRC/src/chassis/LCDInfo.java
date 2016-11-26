package chassis;

import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import utilities.Odometer;
import utilities.Search;

/**
 * Methods and display thread for the EV3 LCD
 * Displays information about robot state and position.
 * @version 3.0
 * @author juliette
 * 
 */
public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private static TextLCD LCD;
	
	public String line1;
	public String line2;
	
	// arrays for displaying data
	private double [] pos;
	
	/**
	 * Constructor for LCDInfo
	 * @param odometer Odometer object
	 * @param LCD TextLCD object
	 * @param start if the runtime display should begin on creation
	 */
	public LCDInfo(Odometer odometer, TextLCD LCD, boolean start) {
		this.odo = odometer;
		LCDInfo.LCD = LCD;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		this.line1 = "";
		this.line2 = "";
		
		// start the timer
		if(start) lcdTimer.start();
	}
	
	/**
	 * Displays user message on line 3 of the LCD
	 * @param message message to display
	 */
	public static void displayMessage(String message) {
		LCD.clear(3);
		LCD.drawString(message, 0, 3);
	}
	
	/**
	 * Displays position and heading, robot state, and other messages
	 * {@inheritDoc}
	 */
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)(pos[2] * 180.0 / Math.PI), 3, 2);
		
		String stateString = Main.state.toString();
		String searchString = Search.searchState.toString();
		LCD.drawString("Robot : " + stateString, 0, 3);
		LCD.drawString("Search: " + searchString, 0, 4);
		
		LCD.drawString(line1, 0, 5);
		LCD.drawString(line2, 0, 6);
	}
	
	/**
	 * Stop LCD thread updates
	 */
	public void pause() {
		lcdTimer.stop();
	}
	
	/**
	 * Restart LCD thread updates
	 */
	public void resume() {
		lcdTimer.start();
	}
	
	/**
	 * Gets TextLCD object given in constructor
	 * @return TextLCD object
	 */
	public static TextLCD getLCD() {
		return LCD;
	}
	
	/**
	 * Display custom string on line 1
	 * @param line1 String to display on user line 1
	 */
	public void setLine1(String line1) {
		this.line1 = line1;
	}
	
	/**
	 * Display custom string on line 2
	 * @param line2 String to display on user line 2
	 */
	public void setLine2(String line2) {
		this.line2 = line2;
	}
}
