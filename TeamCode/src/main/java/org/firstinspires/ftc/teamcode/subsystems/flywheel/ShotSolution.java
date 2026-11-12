package org.firstinspires.ftc.teamcode.subsystems.flywheel;

public final class ShotSolution {
	private final double targetRPM;
	private final double heading;


	public ShotSolution(double targetRPM, double heading) {
		this.targetRPM = targetRPM;
		this.heading = heading;
	}


	public double getTargetRPM() {
		return targetRPM;
	}


	public double getHeading() {
		return heading;
	}


	public boolean isValid() {
		return targetRPM > 0;
	}
}