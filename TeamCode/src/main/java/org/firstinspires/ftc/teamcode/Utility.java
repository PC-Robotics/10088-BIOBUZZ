package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Utility {
	private Utility() {
		throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
	}


	/**
	 * Normalize each value in the powers array if any of the values are greater than 1.
	 * This makes sure that the motors won't receive a |value| > 1.0
	 *
	 * @param powers the array powers for all motors
	 *               modifies the array in place (thanks java)
	 */
	public static void normalizePowers(double[] powers) {
		// no need for check for 0 length array since length is given
		double max = Math.abs(powers[0]);
		for (int i = 1; i < powers.length; i++) {
			max = Math.max(max, Math.abs(powers[i]));
		}

		// normalize values to range [-1, 1]
		if (max > 1.0) {
			for (int i = 0; i < powers.length; i++) {
				powers[i] /= max;
			}
		}
	}


	/**
	 * apply a deadzone to an input.
	 * If the input is between [-DEADZONE_THRESHOLD, THRESHOLD], return 0.
	 *
	 * @param value              the value of the joystick input
	 * @param DEADZONE_THRESHOLD the deadzone that the pad value will be filtered by
	 * @return the joystick value with the deadzone filter applied
	 */
	public static double applyDeadzone(double value, double DEADZONE_THRESHOLD) {
		if (Math.abs(value) > DEADZONE_THRESHOLD) {
			return value;
		}

		return 0;
	}


	/**
	 * Clamp a value between a minimum and maximum value.
	 *
	 * @param value the value to clamp
	 * @param min   the minimum value
	 * @param max   the maximum value
	 * @param <T>   the type of the value
	 * @return the clamped value
	 */
	public static <T extends Comparable<T>> T clamp(T value, T min, T max) {
		if (min.compareTo(max) > 0) {
			throw new IllegalArgumentException("min cannot be greater than max");
		}

		if (value.compareTo(min) < 0) {
			return min;
		} else if (value.compareTo(max) > 0) {
			return max;
		} else {
			return value;
		}
	}


	/**
	 * Retrieves the velocity of a DcMotorEx motor as rpm, the primary measurement
	 * seen in the real world.
	 *
	 * @param motor a DcMotorEx instance
	 * @return the velocity of the motor in RPM
	 */
	public static double getMotorVelocityRPM(DcMotorEx motor) {
		return (Math.abs(motor.getVelocity(AngleUnit.RADIANS)) / (2.0 * Math.PI)) * 60.0;
	}


	/**
	 * Performs linear interpolation between two values.
	 *
	 * @param v0 the starting value
	 * @param v1 the ending value
	 * @param t  the interpolation factor (0.0 to 1.0)
	 * @return the interpolated value
	 */
	public static double lerp(double v0, double v1, double t) {
		return (1 - t) * v0 + t * v1;
	}


	/**
	 * Calculates the distance and angle from one Pose to another.
	 *
	 * @param p1 the starting Pose
	 * @param p2 the target Pose
	 * @return a double array where [0] is distance in inches, [1] is angle in radians
	 */
	public static double[] polarTo(Pose p1, Pose p2) {
		double dx = p2.getX() - p1.getX();
		double dy = p2.getY() - p1.getY();
		return new double[]{Math.hypot(dx, dy), Math.atan2(dy, dx)};
	}

	/**
	 * Calculates the Euclidean distance between two Pose objects.
	 *
	 * @param p1 the first Pose
	 * @param p2 the second Pose
	 * @return the distance in inches
	 */
	public static double distance(Pose p1, Pose p2) {
		double dx = p2.getX() - p1.getX();
		double dy = p2.getY() - p1.getY();
		return Math.hypot(dx, dy);
	}

	/**
	 * Calculates the angle from one Pose to another.
	 *
	 * @param p1 the starting Pose
	 * @param p2 the target Pose
	 * @return the angle in radians
	 */
	public static double angle(Pose p1, Pose p2) {
		double dx = p2.getX() - p1.getX();
		double dy = p2.getY() - p1.getY();
		return Math.atan2(dy, dx);
	}
}