package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import static org.firstinspires.ftc.teamcode.Utility.polarTo;

import com.pedropathing.math.Pose;

public class ShotCalculatorManual implements ShotCalculator {
	private Pose robotPose;
	private Pose goalPose;

	private double rpm = 2000.0;


	@Override
	public ShotSolution run() {
		if (robotPose == null || goalPose == null) {
			return new ShotSolution(0.0, 0.0);
		}

		double heading = polarTo(robotPose, goalPose)[1];

		return new ShotSolution(rpm, heading);
	}


	@Override
	public void init() {
		reset();
	}


	@Override
	public void reset() {
		robotPose = null;
		goalPose = null;
	}


	@Override
	public void updateRobotPose(Pose robotPose) {
		this.robotPose = robotPose;
	}


	@Override
	public void updateGoalPose(Pose goalPose) {
		this.goalPose = goalPose;
	}


	public double getRPM() {
		return rpm;
	}


	public void setRPM(double rpm) {
		this.rpm = Math.max(0.0, rpm);
	}


	public void adjustRPM(double delta) {
		setRPM(rpm + delta);
	}
}
