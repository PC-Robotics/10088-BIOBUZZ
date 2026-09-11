package org.firstinspires.ftc.teamcode.subsystems.shooting;

import com.pedropathing.geometry.Pose;

public interface ShotCalculator {
	void init();

	ShotSolution run();

	void reset();

	void updateRobotPose(Pose robotPose);

	void updateGoalPose(Pose goalPose);

	default void updateRobotVelocity(double vx, double vy, double omega) {
		// optional
	}
}