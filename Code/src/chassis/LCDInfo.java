package chassis;

import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import utilities.Odometer;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private static TextLCD LCD;
	
	public String line1;
	public String line2;
	
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo, TextLCD LCD, boolean start) {
		this.odo = odo;
		LCDInfo.LCD = LCD;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		this.line1 = "";
		this.line2 = "";
		
		// start the timer
		if(start) lcdTimer.start();
	}
	
	public static void displayMessage(String message) {
		LCD.clear(3);
		LCD.drawString(message, 0, 3);
	}
	
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		//LCD.drawString("US: " + odo.getFilteredData(), 0, 0);
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)(pos[2] * 180.0 / Math.PI), 3, 2);
		String stateString;
		switch(Main.state) {
		case k_Capture:
			stateString = "Capture";
			break;
		case k_Disabled:
			stateString = "Disabled";
			break;
		case k_Localization:
			stateString = "Localization";
			break;
		case k_Search:
			stateString = "Search";
			break;
		case k_Setup:
			stateString = "Setup";
			break;
		case k_Avoiding:
			stateString = "Avoiding";
			break;
		default:
			stateString = "";
			break;
		}
		LCD.drawString(stateString, 0, 4);
		LCD.drawString(line1, 0, 5);
		LCD.drawString(line2, 0, 6);
	}
	
	public void pause() {
		lcdTimer.stop();
	}
	
	public void resume() {
		lcdTimer.start();
	}
	
	public static TextLCD getLCD() {
		return LCD;
	}
	
	public void setLine1(String line1) {
		this.line1 = line1;
	}
	
	public void setLine2(String line2) {
		this.line2 = line2;
	}
}
