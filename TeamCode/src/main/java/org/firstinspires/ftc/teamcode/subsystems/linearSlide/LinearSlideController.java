package org.firstinspires.ftc.teamcode.subsystems.linearSlide;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.Utility;

import java.util.function.DoubleSupplier;


public class LinearSlideController {
	// TODO - tune like everything
	public enum Position {
		FLOOR(0), // tune
		DROP(0); // tune

		final int ticks;


		Position(int ticks) {
			this.ticks = ticks;
		}
	}


	public static final double FLOOR_POWER = -0.5; // power while going to floor
	public static final int FLOAT_POSITION = 0; // below this point we float to floor

	// PIDF
	public static double kP = 0.0; // tune
	public static double kD = 0.0; // tune

	private static final double KG_EMPTY = 0.0; // kG with no balls // tune
	private static final double KG_FULL = 0.0; // kg with 4 nectar // tune
	private static final double AMPS_EMPTY = 0.0; // amps with no balls // tune
	private static final double AMPS_FULL = 0.0; // amps with 4 nectar // tune
	private static final double KG_LEEWAY = 0.15; // fraction of slack allowed past the empty/full bounds
	private static final double KG_MIN = KG_EMPTY - (KG_EMPTY * KG_LEEWAY);
	private static final double KG_MAX = KG_FULL + (KG_FULL * KG_LEEWAY);
	private double kG = 0.0; // dont tune this

	// settling
	private static final int SETTLE_POS_TOLERANCE = 0; // tune
	private static final double SETTLE_VELOCITY = 0.0; // tune
	private static final int SETTLE_TIME_THRESHOLD = 0; // tune
	private boolean settled = false;
	private boolean settleCandidate = false;
	private final Timer settleTimer = new Timer();
	private final Utility.LowpassFilter currentFilter = new Utility.LowpassFilter(5);

	private int error = 0;


	public double update(Position position, int currentPosition, double velocity, DoubleSupplier current) {
		if (position == Position.FLOOR) {
			error = 0;
			settleCandidate = false;
			settled = false;
			return currentPosition > FLOAT_POSITION ? FLOOR_POWER : 0.0;
		}

		error = position.ticks - currentPosition;
		double power = kP * error - kD * velocity + kG;

		updateSettled(velocity); // error is class variable
		if (settled) {
			updateGravityFeedforward(current); // for next loop
		}

		return power;
	}


	private void updateSettled(double velocity) {
		boolean sus = Math.abs(error) < SETTLE_POS_TOLERANCE
				&& Math.abs(velocity) < SETTLE_VELOCITY;

		if (sus) {
			if (!settleCandidate) {
				settleCandidate = true;
				settleTimer.reset();
			}
			settled = settleTimer.milliseconds() > SETTLE_TIME_THRESHOLD;
		} else {
			settleCandidate = false;
			settled = false;
		}
	}


	private void updateGravityFeedforward(DoubleSupplier current) {
		double amps = currentFilter.update(current.getAsDouble());
		double t = Utility.inverseLerp(AMPS_EMPTY, AMPS_FULL, amps);
		kG = Utility.clamp(Utility.lerp(KG_EMPTY, KG_FULL, t), KG_MIN, KG_MAX);
	}


	public void reset() {
		settleCandidate = false;
		settled = false;
		error = 0;
	}


	public int getError() {
		return error;
	}


	public double getKg() {
		return kG;
	}


	public boolean isSettled() {
		return settled;
	}
}
