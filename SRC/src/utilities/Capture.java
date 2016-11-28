package utilities;

import chassis.Main;
import chassis.Main.RobotState;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Odometer.LINEDIR;
import utilities.Search.SearchState;

/**
 * Thread that runs when capturing a styrofoam block
 * @version 3.0
 * 
 *
 */
public class Capture extends Thread {
	
	// instances
	private Odometer odo;
	private Navigation nav;
	private Search search;
	
	// coordinates
	private double[][] GREEN;
	private double[] towerPosition;
	
	private static double[] cardinalPoint;
	
	// states
	public enum CaptureState {Grab, Return, Stack, Idle};
	public static CaptureState captureState = CaptureState.Idle;

	private EV3LargeRegulatedMotor clawMotor;
	
	private int towerHeight;
	
	/**
	 * Capture Thread Constructor
	 * @param odometer Odometer Object
	 * @param GREEN green scoring zone coordinates
	 */
	public Capture(Odometer odometer, Search search, double[][] GREEN) {
		this.odo = odometer;
		this.nav = new Navigation(this.odo);
		this.search = search;
		this.GREEN = GREEN;
		this.towerHeight = 0;
		this.towerPosition = new double[]{(GREEN[0][0] + GREEN[1][0])/2,(GREEN[0][1] + GREEN[1][1])/2};
	}
	
	/**
	 * start Capture thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		//DropCheck dc = new DropCheck(this.odo, this.search);
		///dc.start();
		while(true) {
			if(Main.state == RobotState.Avoiding) Capture.captureState = CaptureState.Idle;
			
			switch(captureState) {
			case Grab:
				Main.forklift.liftDown(); //Descend forklift. 
				Main.forklift.grip();
				Main.forklift.liftUp();
				captureState = CaptureState.Return; //Move on to Return
				break;
			case Return:
				DropCheck dropChecker = new DropCheck(odo, search);
				dropChecker.start();
				odo.moveCM(Odometer.LINEDIR.Backward, 3, true); //Back up to avoid bumping into things when spinning
				nav.travelToInterruptible(cardinalPoint[0], cardinalPoint[1], dropChecker.interrupt); //Travel back to last cardinal point, this path is guaranteed to be clear
			
				while(!dropChecker.interrupt.tryLock());	//unlocks when done
				dropChecker.interrupt.unlock();
				
				if(dropChecker.getDropped() == false) { //block was not dropped			
					if(towerHeight == 0){
						//If towerHeight is 0, travel to a predetermined stacking area and drop the block
						nav.travelTo(towerPosition[0], towerPosition[1]);
						while(Navigation.PathBlocked) {
							Avoider.avoidState = Avoider.AvoidState.Enabled;
							try { Thread.sleep(2*Util.SLEEP_PERIOD); } catch(Exception ex) {}
							while(Main.state == RobotState.Avoiding) {
								try { Thread.sleep(2*Util.SLEEP_PERIOD); } catch(Exception ex) {}
							}
							nav.travelTo(towerPosition[0],towerPosition[1]);
						}
						captureState = CaptureState.Stack;
						odo.moveCM(Odometer.LINEDIR.Backward, Util.CLAW_TO_CENTER, true);
		
						break;
					}
					//If there is already a tower, travel towards towerPosition until the tower is detected.
					double targetHeading = Math.atan2(-odo.getY() + towerPosition[1],-odo.getX() + towerPosition[0]);
					nav.turnTo(targetHeading,true); //Turn to face tower position, stop motors
					odo.setMotorSpeed(Odometer.NAVIGATE_SPEED); //Move forward until the tower is detected.
					
					double[] approachPos = new double[3];
					double scoringDiagnol = Odometer.euclideanDistance(GREEN[0], GREEN[1]);	//max distance from point and the tower (diagnol of zone)
					boolean reachedTower = true;

					odo.getPosition(approachPos);	//where it started the approach
					odo.forwardMotors();

					while(Main.usSensor.getMedianSample(Util.US_SAMPLES) > Util.TOWER_DISTANCE &&
							(reachedTower = Odometer.euclideanDistance(new double[] {odo.getX(), odo.getY()}, approachPos) < scoringDiagnol));
					odo.stopMotors();
					if(!reachedTower) {
						towerHeight = 0;	//will create new tower
					} else {
						captureState = CaptureState.Stack;
					}
				}
				dropChecker.interrupt();
				break;
			case Stack:
				Main.forklift.liftToTower(towerHeight++); //Descend lift to height of tower, increase towerHeight
				Main.forklift.ungrip();
				Main.forklift.liftUp();
				if(Search.searchState != SearchState.Default) Search.searchState = SearchState.AtDropZone; //Pass control back to search
				captureState = CaptureState.Idle;
				break;
			case Idle:
				// idle state, waiting for avoidance to return
				try{
					Thread.sleep(Util.SLEEP_PERIOD);
				} catch(Exception e) {}
				break;
			default:
				break;
			}
		}
	}
	
	/**
	 * Checks if the position is in field bounds
	 * @param x
	 * @param y
	 * @param width field width (x)
	 * @param height field height (y)
	 * @return
	 */
	public static boolean inBounds(double x,double y, double width, double height) {
		return (x < width) && (y < height) && x > 0 && y > 0;
	}
	
	// sets the current cardinal point at which the robot is bound to
	public static void setContext(double[] cardinalPoint) {
		Capture.cardinalPoint = cardinalPoint;
	}
}
