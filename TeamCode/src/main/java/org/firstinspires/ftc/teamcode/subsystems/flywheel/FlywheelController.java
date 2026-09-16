package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import static java.lang.Math.signum;

import com.pedropathing.controllers.filters.KalmanFilter;

public class FlywheelController {
	public enum State {
		SPINUP,
		READY,
		COAST,
	}


	public static final double MAX_POWER = 0.98;

	// thresholds for state transitions
	public static final double READY_ENTER = 0.95;
	public static final double READY_EXIT = 0.9;
	public static final double COAST_ENTER = 1.1;
	public static final double COAST_EXIT = 1.05;

	// kalman filter
	public static final double STATE_STDDEV = 2;
	public static final double MEASUREMENT_STDDEV = 0.2;

	// TODO - tune
	public static double kP = 0.0;
	public static double kV = 0.0;
	public static double kS = 0.0;

	private final KalmanFilter kalmanFilter;
	private double target = 0.0;
	private double error = 0.0;
	private State controllerState = State.SPINUP;


	public FlywheelController() {
		kalmanFilter = new KalmanFilter(STATE_STDDEV, MEASUREMENT_STDDEV);
	}


	public double update(double measuredRPM) {
		// update kalman filter
		kalmanFilter.update(measuredRPM);
		double filteredRPM = kalmanFilter.state();


		// update error
		error = target - filteredRPM;

		updateState(filteredRPM);
		return calculate();
	}


	private void updateState(double filteredRPM) {
		switch (controllerState) {
			case SPINUP:
				if (filteredRPM >= target * READY_ENTER) {
					controllerState = State.READY;
				}
				break;
			case READY:
				if (filteredRPM < target * READY_EXIT) {
					controllerState = State.SPINUP;
				} else if (filteredRPM > target * COAST_ENTER) {
					controllerState = State.COAST;
				}
				break;
			case COAST:
				if (filteredRPM < target * COAST_EXIT) {
					controllerState = State.READY;
				}
				break;
		}
	}


	private double calculate() {
		switch (controllerState) {
			case SPINUP:
				return MAX_POWER;
			case READY:
				return Math.max(0.0, Math.min(MAX_POWER, kV*target + kP*error + kS*signum(error)));
			case COAST:
			default:
				return 0.0; // coast
		}
	}


	public void setTarget(double targetRPM) {
		target = Math.max(0.0, targetRPM);
	}

	public void reset(double measuredRPM) {
		kalmanFilter.reset(measuredRPM, 1.0, 1.0);
		error = 0.0;
		controllerState = State.SPINUP;
	}

	public State getState() {
		return controllerState;
	}

	public double getError() {
		return error;
	}

	public double getFilteredRPM() {
		return kalmanFilter.state();
	}
}
