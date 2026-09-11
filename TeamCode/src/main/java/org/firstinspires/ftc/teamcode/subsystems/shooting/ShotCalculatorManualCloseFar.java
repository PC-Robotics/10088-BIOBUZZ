package org.firstinspires.ftc.teamcode.subsystems.shooting;

import static org.firstinspires.ftc.teamcode.Utility.polarTo;

import com.pedropathing.geometry.Pose;

public class ShotCalculatorManualCloseFar implements ShotCalculator {
	public enum ShotPreset {
		IDLE(0.0, false),
		CLOSE(3000.0, true),
		FAR(5500.0, true);

		private final double rpm;
		private final boolean valid;

		ShotPreset(double rpm, boolean valid) {
			this.rpm = rpm;
			this.valid = valid;
		}

		public double getRPM() {
			return rpm;
		}

		public boolean isValid() {
			return valid;
		}
	}


	private Pose robotPose;
	private Pose goalPose;

	private ShotPreset preset = ShotPreset.IDLE;

	public ShotCalculatorManualCloseFar() {
	}

	@Override
	public ShotSolution run() {
		if (robotPose == null || goalPose == null) {
			return new ShotSolution(0.0, 0.0, 0.0, false);
		}

		double[] polar = polarTo(robotPose, goalPose);
		double distance = polar[0];
		double heading = polar[1];

		return new ShotSolution(distance, preset.getRPM(), heading, preset.isValid());
	}

	@Override
	public void init() {
		reset();
	}

	@Override
	public void reset() {
		robotPose = null;
		goalPose = null;
		preset = ShotPreset.IDLE;
	}

	@Override
	public void updateRobotPose(Pose robotPose) {
		this.robotPose = robotPose;
	}

	@Override
	public void updateGoalPose(Pose goalPose) {
		this.goalPose = goalPose;
	}

	public void setPreset(ShotPreset preset) {
		this.preset = preset;
	}

	public ShotPreset getPreset() {
		return preset;
	}
}