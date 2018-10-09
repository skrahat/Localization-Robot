package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Odometer;
import ca.mcgill.ecse211.lab3.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

/**
 * This class is used to let the robot navigate to the points supplied in the Driver class
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is used to drive the robot to the waypoints while avoiding blocks
 */

public class Navigation {
	// Motor Objects, Odometer and related variables
	private Odometer odometer;
	private double dX, dY;
	private double theta;
	public static double distance;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 130;
	public static double WHEEL_RAD;
	public static double TRACK;
	public static final double TILE_SIZE = 30.48;
	private static double[] currentPosition;

	private static final EV3LargeRegulatedMotor leftMotor = Lab3.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab3.rightMotor;

	private static boolean isNavigating;
	private static SampleProvider usDistance = Lab3.usDistance;
	private static float[] usData = Lab3.usData;

	private static double angle = 0; // lab3
	private static double hypo = 0; // lab3
	private static int XTemp = 0;
	private static int YTemp = 0;
	private static int Xaxis = 1;
	private static int Yaxis = 1;
	private static final double rightRadius = 2.2;
	private static final double leftRadius = 2.2;
	public static final double track = 15.75;
	private static Odometer odo;
	public static int obstDistance;
	public static double Yleft;
	public static double Yinit;
	public static double Xleft;
	public static double Xinit;

	public static double distanceLeft;

	/**
	 * Constructor for ObstacleNav. Receives sensor information, motors, and related
	 * parameters from lab3 class
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param WHEEL_RAD
	 * @param TRACK
	 * @param sample
	 * @param data
	 * @throws OdometerExceptions
	 */

	public static void travelTo(double x, double y) {
		isNavigating = true;

		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		Xaxis = (int) x;
		Yaxis = (int) y;
		XTemp = Xaxis - Lab3.XCor;
		YTemp = Yaxis - Lab3.YCor;
		hypo = Math.sqrt((XTemp * XTemp) + (YTemp * YTemp));

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		currentPosition = odo.getXYT();
		double deltaX = x * TILE_SIZE - currentPosition[0];
		double deltaY = y * TILE_SIZE - currentPosition[1];
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		// leftMotor.rotate(convertDistance(leftRadius, distanceLeft), true);
		// rightMotor.rotate(convertDistance(rightRadius, distanceLeft), false);

		if (YTemp != 0) {
			angle = Math.atan(XTemp / YTemp); // lab3
		}

		hypo = Math.sqrt((XTemp * XTemp) + (YTemp * YTemp));
		angle = Math.toDegrees(angle);
		angle = Math.abs(angle);

		if (XTemp < 0 && YTemp < 0) {
			angle = angle + 180;
		}

		else if (XTemp > 0 && YTemp < 0) {
			angle = angle + 90;
		}

		else if (XTemp < 0 && YTemp > 0) {
			angle = angle + 270;
		}

		if (XTemp == 0 && YTemp < 0) {
			angle = 180;
		} else if (XTemp == 0 && YTemp > 0) {
			angle = 0;
		} else if (XTemp < 0 && YTemp == 0) {
			angle = 270;
		} else if (XTemp > 0 && YTemp == 0) {
			angle = 90;
		}

		if (Lab3.ZCor > 0) {
			angle = angle - Lab3.ZCor;
		}

		if (Lab3.ZCor < 0) {
			angle = angle + Lab3.ZCor;
		}

		if (angle > 180) {
			angle = angle - 360;
		}
		if (angle < -180) {
			angle = angle + 360;
		}
		turnTo(angle);

		Lab3.XCor = Xaxis;
		Lab3.YCor = Yaxis;
		Lab3.ZCor = odo.getXYT()[2];

		leftMotor.forward();
		rightMotor.forward();

		while (isNavigating) {

			usDistance.fetchSample(usData, 0); // acquire data
			obstDistance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			currentPosition = odo.getXYT();

			deltaX = x * TILE_SIZE - currentPosition[0];
			deltaY = y * TILE_SIZE - currentPosition[1];
			distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

			if (obstDistance < 6) {
				Sound.beep();

				leftMotor.stop(true);
				rightMotor.stop();

				leftMotor.rotate(convertAngle(leftRadius, track, 90), true); // lab3 rotates
				rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);

				leftMotor.rotate(convertDistance(leftRadius, 25), true);
				rightMotor.rotate(convertDistance(rightRadius, 25), false);

				leftMotor.rotate(convertAngle(leftRadius, track, -90), true); // lab3 rotates
				rightMotor.rotate(-convertAngle(rightRadius, track, -90), false);

				leftMotor.rotate(convertDistance(leftRadius, 38), true);
				rightMotor.rotate(convertDistance(rightRadius, 38), false);

				leftMotor.rotate(convertAngle(leftRadius, track, -90), true); // lab3 rotates
				rightMotor.rotate(-convertAngle(rightRadius, track, -90), false);

				leftMotor.rotate(convertDistance(leftRadius, 25), true);
				rightMotor.rotate(convertDistance(rightRadius, 25), false);

				leftMotor.rotate(convertAngle(leftRadius, track, 90), true); // lab3 rotates
				rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);

				leftMotor.forward();
				rightMotor.forward();

			}

			if (distance < 2.0) {
				leftMotor.stop(true);
				rightMotor.stop();
				isNavigating = false;

			}

		}
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public static void turnTo(double theta) {
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		leftMotor.rotate(convertAngle(leftRadius, track, theta), true); // lab3 rotates
		rightMotor.rotate(-convertAngle(rightRadius, track, theta), false); // lab3

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

	}

}
////////////////////////