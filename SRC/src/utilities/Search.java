package utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import chassis.ColorSensor;
import chassis.Main;
import chassis.Main.RobotState;
import chassis.USSensor;
import utilities.Avoider.AvoidState;
import utilities.Capture.CaptureState;
import utilities.Odometer.TURNDIR;

/**
 * Blue styrofoam block search functionality for the robot
 * 
 * @author juliette
 * @version 0.2
 * 
 * 
 */
/*
 * added rgb vectors
 * 
 * @author Jackson
 * 
 * @version 0.3
 */
public class Search extends Thread {

	// Instances
	private Odometer odo;
	private Navigation nav;
	private USSensor usSensor;
	private ColorSensor colorSensor;

	// Coordinates
	private double[] N = new double[2];
	private double[] W = new double[2];
	private double[] S = new double[2];
	private double[] E = new double[2];
	private double[][] cardinals = new double[4][2];
	private int currCardinal;

	// object detection
	private ArrayList<double[]> objectLocations = new ArrayList<double[]>();
	private float lastDistanceDetected;

	// states
	public enum SearchState {
		Default, AtCardinal, AtDropZone, Inspecting, Iddle
	};

	public static SearchState searchState;
	// color vectors at different distances
	private static final double[] colorVectorFar = { 0.114832, 0.160879,
			0.159646 };
	private static final double[] colorVectorClose = { 0.023529, 0.054678,
			0.139235 };
	private static final double[] colorVectorMedium = { 0.194117, 0.255294,
			0.209411 };
	// threshold for blue foam
	private static final int blueFoamThreshold = 9;

	// TODO TODO TODO TODO
	// - when latching angle for detected object, make sure it doesn't latch it
	// multiple times -> give a min interval
	// - at this state code would only avoid 1 object, handle dynamic obstacle
	// avoidance

	/**
	 * Constructor for Search Class
	 * 
	 * @param odometer
	 *            - Odometer object
	 * @param colorSensor
	 *            - ColorSensor object
	 * @param usSensor
	 *            - USSensor object
	 * @param GREEN
	 *            - coordinates of the green scoring zone
	 * 
	 * @author Juliette Regimbal
	 * @version
	 */
	public Search(Odometer odometer, ColorSensor colorSensor,
			USSensor usSensor, double[][] GREEN) {
		this.odo = odometer;
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		nav = new Navigation(odo);

		Search.searchState = SearchState.Default;

		// mid points of the GREEN box
		double midX = (GREEN[0][0] + GREEN[1][0]) / 2;
		double midY = (GREEN[0][1] + GREEN[1][1]) / 2;

		// cardinal search points
		this.S = new double[] { midX, GREEN[0][1] };
		this.N = new double[] { midX, GREEN[1][1] };
		this.W = new double[] { GREEN[0][0], midY };
		this.E = new double[] { GREEN[1][0], midY };

		this.cardinals = new double[][] { N, W, S, E };

		this.lastDistanceDetected = -1;
		this.currCardinal = -1;
	}

	/**
	 * Starts search thread {@inheritDoc}
	 */
	@Override
	public void run() {
		while (Main.state != Main.RobotState.Search) {
			// check if interrupted
			if (Thread.interrupted())
				return;
			// wait for localization to finish
			try {
				Thread.sleep(300);
			} catch (Exception e) {
			}
		}

		isStyrofoamBlock(); // Initialize rgb mode

		while (true) {
			switch (searchState) {

			case Default:
				// at origin, not yet at GREEN zone
				if (currCardinal == -1) {

					// in case we encounter foam on our way to GREEN zone
					// Capture.moveForklift(FOrkliftPosition.Ground)

					// next cardinal, wrap around
					currCardinal++;
					currCardinal %= 4;

					nav.travelTo(cardinals[currCardinal][0],
							cardinals[currCardinal][1]);

					// verify if navigation was interrupted
					if (Navigation.PathBlocked) {
						// does the robot already have a block?
						if (Capture.captureState == CaptureState.Disabled)
							inspectObject();
						else {
							Avoider.avoidState = AvoidState.Enabled;
							// wait for avoider to finish
							while (Main.state == RobotState.Avoiding)
								;
						}

					}

					Search.searchState = SearchState.AtCardinal;

				} else {

					// no more object at that cardinal point
					if (objectLocations.isEmpty()) {
						// next cardinal, wrap around
						currCardinal++;
						currCardinal %= 4;

						// linear set of instructions to reach next cardinal
						// at this step the robot is ensured to be on the old
						// cardinal point

						// place on correct heading
						nav.turnTo(
								(Math.PI + (currCardinal - 1) * Math.PI / 2),
								true);

						// travel until it reaches the axis of next cardinal
						// point
						travelToAxis(true);

						// travel to the cardinal point without fear of bumping
						// onto the tower
						nav.travelTo(cardinals[currCardinal][0],
								cardinals[currCardinal][1]);

						Search.searchState = SearchState.AtCardinal;
					} else {
						Search.searchState = SearchState.Inspecting;
					}
				}

				Avoider.avoidState = AvoidState.Disabled;

				break;

			case AtCardinal:

				// turn to start scanning angle
				double startAngle = Math.PI / 2 * currCardinal;
				nav.turnTo(startAngle, true);

				// reinitialize last distance
				this.lastDistanceDetected = -1;

				// start scatter search
				odo.setMotorSpeed(Util.MOTOR_SLOW);
				odo.spin(TURNDIR.CCW);
				double targetAngle = startAngle + Math.PI;
				while (Math.abs(Navigation.minimalAngle(odo.getTheta(),
						targetAngle)) > Util.SCAN_THETA_THRESHOLD) {
					// object found
					if (isObjectDetected()) {
						odo.stopMotors();

						// latch distance to object
						double distance = usSensor
								.getMedianSample(Util.US_SAMPLES);
						objectLocations.add(new double[] { distance,
								odo.getTheta() });

						// resume scatter search
						odo.setMotorSpeed(Util.MOTOR_SLOW);
						odo.spin(TURNDIR.CCW);
					}
				}

				odo.stopMotors();

				// sort list of object locations
				Collections.sort(objectLocations, new Comparator<double[]>() {
					@Override
					public int compare(double[] location1, double[] location2) {
						if (location1[0] == location2[0]) {
							return 0;
						} else {
							return location1[0] > location2[0] ? 1 : -1;
						}
					}
				});

				searchState = SearchState.Inspecting;

				break;

			case AtDropZone:

				// back off until to avoid colliding with tower
				travelToAxis(false);

				searchState = SearchState.Default;

				break;

			case Inspecting:

				if (!objectLocations.isEmpty()) {
					// examine closest object
					double heading = objectLocations.get(0)[1];
					objectLocations.remove(0);
					nav.turnTo(heading, true);

					inspectObject();
				} else {
					searchState = SearchState.Default;
				}

				break;

			case Iddle:

				// iddle state, waiting for capture to return and stack block
				try {
					Thread.sleep(Util.SLEEP_PERIOD);
				} catch (Exception e) {
				}
				break;
			}
		}

	}

	// travel to correspondent axis of current cardinal point
	private void travelToAxis(boolean frontwards) {

		String axis = (currCardinal % 2 == 0) ? "Y" : "X";

		odo.setMotorSpeeds(Util.MOTOR_FAST, Util.MOTOR_FAST);
		if (frontwards)
			odo.forwardMotors();
		else
			odo.backwardMotors();

		switch (axis) {
		case "X":
			while (odo.getX() < cardinals[currCardinal][0] - Util.CM_TOLERANCE
					|| odo.getX() > cardinals[currCardinal][0]
							+ Util.CM_TOLERANCE)
				;
			break;
		case "Y":
			while (odo.getY() < cardinals[currCardinal][1] - Util.CM_TOLERANCE
					|| odo.getY() > cardinals[currCardinal][1]
							+ Util.CM_TOLERANCE)
				;
			break;
		}

		odo.stopMotors();

	}

	/**
	 * When object is in sight, approach slowly and inspect
	 */
	private void inspectObject() {
		odo.setMotorSpeed(Util.MOTOR_SLOW);
		odo.forwardMotors();

		// wait until close enough to determine if it's a styrofoam block
		while (usSensor.getMedianSample(Util.US_SAMPLES) > Util.BLOCK_DISTANCE)
			;
		odo.stopMotors();

		// inspect object
		if (isStyrofoamBlock()) {
			searchState = SearchState.Iddle;
			Main.state = Main.RobotState.Capture;
		} else {
			// backoff a little to avoid hitting the block while turning
			odo.setMotorSpeed(Util.MOTOR_SLOW);
			odo.backwardMotors();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			// return to cardinal point and inspect next object
			nav.travelTo(cardinals[currCardinal][0], cardinals[currCardinal][1]);
			searchState = SearchState.Default;
		}
	}

	/**
	 * 
	 * @return if the detected object is a styrofoam block
	 */
	// TODO use complete RGB vector to compare
	private boolean isStyrofoamBlock() {
		return (int) (calculateColorDis(colorSensor.getColor())) <= blueFoamThreshold;
	}

	/**
	 * 
	 * @return if there is an object that can be detected by the robot
	 */
	private boolean isObjectDetected() {
		float currentDistance = usSensor.getMedianSample(Util.US_SAMPLES);

		// at each iteration of scatter search, last distance is reset
		if (this.lastDistanceDetected < 0)
			this.lastDistanceDetected = currentDistance;

		// avoids latching the same object multiple times by creating a +/-5% BW
		// around a detected object
		boolean isObject = (currentDistance < lastDistanceDetected + 5)
				&& (currentDistance > lastDistanceDetected - 5)
				&& (currentDistance <= Util.SEARCH_DISTANCE);
		lastDistanceDetected = currentDistance;
		return isObject;
	}

	/**
	 * Measures distance between two RGB color vectors
	 * 
	 * @param a
	 *            first RGB vector
	 * @param b
	 *            second RGB vector
	 * @return magnitude of the distance between a and b
	 */
	private static double calculateColorDis(float[] rgb) {
		double close = Math.sqrt(Math.pow(rgb[0] - colorVectorClose[0], 2)
				+ Math.pow(rgb[1] - colorVectorClose[1], 2)
				+ Math.pow(rgb[2] - colorVectorClose[2], 2));
		double far = Math.sqrt(Math.pow(rgb[0] - colorVectorFar[0], 2)
				+ Math.pow(rgb[1] - colorVectorFar[1], 2)
				+ Math.pow(rgb[2] - colorVectorFar[2], 2));
		double medium = Math.sqrt(Math.pow(rgb[0] - colorVectorMedium[0], 2)
				+ Math.pow(rgb[1] - colorVectorMedium[1], 2)
				+ Math.pow(rgb[2] - colorVectorMedium[2], 2));

		if (close < far && close < medium)
			return close * 100;
		if (medium < far && medium < close)
			return medium * 100;
		if (far < close && far < medium)
			return far * 100;
		return (close <= far ? close <= medium ? close : medium : far) * 100;
	}
}