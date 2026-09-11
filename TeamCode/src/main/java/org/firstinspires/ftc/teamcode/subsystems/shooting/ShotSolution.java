package org.firstinspires.ftc.teamcode.subsystems.shooting;

public final class ShotSolution {
	private final double distance;
	private final double targetRPM;
	private final double heading;
	private final boolean valid;

	public ShotSolution(double distance, double targetRPM, double heading, boolean valid) {
		this.distance = distance;
		this.targetRPM = targetRPM;
		this.heading = heading;
		this.valid = valid;
	}

	public double getDistance() {
		return distance;
	}

	public double getTargetRPM() {
		return targetRPM;
	}

	public double getHeading() {
		return heading;
	}

	public boolean isValid() {
		return valid;
	}
}