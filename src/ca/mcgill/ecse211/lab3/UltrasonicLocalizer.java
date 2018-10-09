package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer extends Lab3 {

	// Not sure if we should make it as enum or different methods
//public enum LocalizationType  {Falling_edge, Rising_edge};

	private Odometer odometer;
	private Navigation nav;
	private double dX, dY;
	private double theta;
	public static double distance;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 130;
	public static double WHEEL_RAD;
	public static final double TILE_SIZE = 30.48;
	public static double TRACK;
	private static final EV3LargeRegulatedMotor leftMotor = Lab3.leftMotor;
	private static final EV3LargeRegulatedMotor rightMotor = Lab3.rightMotor;

	private static SampleProvider usDistance = Lab3.usDistance;
	private static float[] usData = Lab3.usData;

	private static Odometer odo;
	private static final double rightRadius = 2.2;
	private static final double leftRadius = 2.2;
	public static final double track = 15.75;
	private static boolean isActive;
	private static boolean isActive2;
	private static int edge;
	private static double delta1;
	private static double delta2;
	private static double delta3;
	private static double delta4;

	private static int obstDistance;
	private static double[] currentPosition;

	public static void UltrasonicLocalizerMain(int edge) {

		if (edge == 1) {
			fallingEdge();
		} else {
			risingEdge();
		}

	}

	public static void fallingEdge() {
		isActive = true;
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		while (isActive) {
			usDistance.fetchSample(usData, 0); // acquire data
			obstDistance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			if (obstDistance > 255) {
				obstDistance = 255;
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();

			if ( obstDistance < 14) {
				leftMotor.stop(true);
				rightMotor.stop();
				currentPosition = odo.getXYT();
				delta1 = currentPosition[2];
				isActive = false;
				isActive2 = true;
				leftMotor.rotate(convertAngle(leftRadius, track, 30), true); // lab3 rotates
				rightMotor.rotate(-convertAngle(rightRadius, track, 30), false);
			}
		}
		while (isActive2) {
			usDistance.fetchSample(usData, 0); // acquire data
			obstDistance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			if (obstDistance > 255) {
				obstDistance = 255;
			}
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();

			if (obstDistance < 14) {
				leftMotor.stop(true);
				rightMotor.stop();
				currentPosition = odo.getXYT();
				delta2 = currentPosition[2];
				isActive2 = false;

			}
		}
		if (delta1 > delta2) {
			delta3 = 45-((delta1 + delta2) / 2);
			odo.setTheta(180 + delta3 + odo.getXYT()[2]);
			currentPosition = odo.getXYT();

			leftMotor.rotate(convertAngle(leftRadius, track, -currentPosition[2]), true); // lab3 rotates
			rightMotor.rotate(-convertAngle(rightRadius, track, -currentPosition[2]), false);
			Sound.beep();
			Sound.beep();

		} else {
			delta3 = 135 - ((delta1 + delta2) / 2);
			odo.setTheta(180 + delta3 + odo.getXYT()[2]);
			currentPosition = odo.getXYT();
			leftMotor.rotate(convertAngle(leftRadius, track, -currentPosition[2]), true); // lab3 rotates
			rightMotor.rotate(-convertAngle(rightRadius, track, -currentPosition[2]), false);
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();

		}
	

	}

	public static void risingEdge() {
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
			return;
		}

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
