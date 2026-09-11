package org.firstinspires.ftc.teamcode.subsystems.shooting;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.polarTo;

import com.pedropathing.geometry.Pose;


/**
 * solve rpm using projectile motion equations.
 * from the range formula
 * deltaHeight = d * tan(theta) - (g * d^2) / (2 * v0^2 * cos^2(theta))
 * we get
 * v0 = sqrt(g * d^2 / (2 * cos^2(theta) * (d*tan(theta) - deltaHeight)))
 * <p>
 * since force from flywheel to ball is not elastic, we account for efficiency
 * surfaceSpeed = v0 / efficiency
 * <p>
 * then we find targetRPM
 * targetRPM = 60 * surfaceSpeed / (2pi * flywheelRadius)
 */
public class ShotCalculatorProportional implements ShotCalculator {
	private static final double GRAVITY = 315.30567; // g (9.8m/s^2) converted to in/s^2

	private Pose robotPose;
	private Pose goalPose;

	private double launchAngleDegrees = 45.0;

	// heights above field floor, inches
	private double launchHeight = 12.0;
	private double goalHeight = 38.0;

	// flywheel stuff
	private double flywheelRadius = 2.0; // inches
	private double efficiency = 0.5;  // fraction of surface speed imparted to the projectile

	// distance constraints, inches
	private double minDistance = 0.0;
	private double maxDistance = 200.0;

	public ShotCalculatorProportional() {
	}

	@Override
	public ShotSolution run() {
		if (robotPose == null || goalPose == null) {
			return new ShotSolution(0.0, 0.0, 0.0, false);
		}

		double[] polar = polarTo(robotPose, goalPose);
		double distance = polar[0];
		double heading = polar[1];

		if (distance < minDistance || distance > maxDistance) {
			return new ShotSolution(distance, 0.0, heading, false);
		}

		// save calculations
		double theta = Math.toRadians(launchAngleDegrees);
		double cos = Math.cos(theta);
		double deltaHeight = goalHeight - launchHeight;

		// denominator for exit velocity formula
		double denominator = 2.0 * cos * cos * (distance * Math.tan(theta) - deltaHeight);
		if (denominator <= 0.0) { // if denominator <= 0 projectile can not reach goal
			return new ShotSolution(distance, 0.0, heading, false);
		}

		double exitVelocity = Math.sqrt((GRAVITY * distance * distance) / denominator); // in/s
		double surfaceSpeed = exitVelocity / efficiency; // in/s
		double targetRPM = (60.0 * surfaceSpeed) / (2.0 * Math.PI * flywheelRadius);

		return new ShotSolution(distance, targetRPM, heading, true);
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

	public double getLaunchAngleDegrees() {
		return launchAngleDegrees;
	}

	public void setLaunchAngleDegrees(double launchAngleDegrees) {
		// in case i am stupid and give it a negative number
		this.launchAngleDegrees = clamp(launchAngleDegrees, 1.0, 89.0);
	}

	public double getLaunchHeight() {
		return launchHeight;
	}

	public void setLaunchHeight(double launchHeight) {
		this.launchHeight = launchHeight;
	}

	public double getGoalHeight() {
		return goalHeight;
	}

	public void setGoalHeight(double goalHeight) {
		this.goalHeight = goalHeight;
	}

	public double getflywheelRadius() {
		return flywheelRadius;
	}

	public void setflywheelRadius(double flywheelRadius) {
		this.flywheelRadius = Math.max(1e-6, flywheelRadius);
	}

	public double getEfficiency() {
		return efficiency;
	}

	public void setEfficiency(double efficiency) {
		// if this goes to zero code gets slimed
		this.efficiency = Math.max(1e-6, efficiency);
	}

	public double getMinDistance() {
		return minDistance;
	}

	public void setMinDistance(double minDistance) {
		this.minDistance = Math.max(0.0, minDistance);
	}

	public double getMaxDistance() {
		return maxDistance;
	}

	public void setMaxDistance(double maxDistance) {
		this.maxDistance = Math.max(0.0, maxDistance);
	}
}
