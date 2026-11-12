package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Poses;
import org.firstinspires.ftc.teamcode.Utility;

public class Drivetrain implements Subsystem {
	public enum Mode {
		MANUAL,
		AUTO_AIM
	}


	private Follower follower;
	private final Mecanum motors;
	private boolean fieldCentric = true;
	private Mode mode = Mode.MANUAL;
	Pose pose;
	Pose anchor;
	Controller headingPID = Controller.pid(0, 0, 0);
	final static double DEADZONE = 0.1;


	public Drivetrain(Follower follower, OpMode opMode) {
		this.follower = follower;
		motors = ((Mecanum) follower.drivetrain);
	}


	public void teleOpDrive(double forward, double strafe, double turn, double speedMultiplier) {
		pose = follower.pose(); // cache ts

		switch (mode) {
			case MANUAL:
				anchor = null;
				DrivePowers powers = new DrivePowers(
						forward * speedMultiplier,
						strafe * speedMultiplier,
						turn * speedMultiplier
				);
				if (fieldCentric) {
					powers = ManualDrive.fieldCentric(powers, pose.heading());
				}
				follower.manual(powers);
				break;
			case AUTO_AIM:
				DrivePowers translational;
				double hiveTheta = Utility.polarTo(pose, Poses.HIVE)[1];

				if (forward * forward + strafe * strafe >= DEADZONE * DEADZONE) { // if driving,
					anchor = null;
					translational = new DrivePowers(
							forward * speedMultiplier,
							strafe * speedMultiplier,
							0
					); // calculate translational powers
					if (fieldCentric) {
						translational = ManualDrive.fieldCentric(translational, pose.heading());
					}
					// drive with heading lock onto hive
					follower.manual(ManualDrive.headingLock(follower, headingPID, translational, hiveTheta));
				} else { // if not driving
					if (anchor == null) { // if we dont have a stopping point
						// pedropathing tells us the predicted stopping point if we were to brake
						Pose brakeDistance = ((Foresight) follower.algorithm()).getBrakeDisplacement(follower.twist(), pose.heading());
						anchor = pose.plus(brakeDistance); // we add that to our current pose to get our anchor
					}
					follower.hold(anchor.withHeading(hiveTheta)); // and then hold with heading lock onto hive
				}
		}
	}


	// placeholder
	@Override
	public Command stop() {
		return instant(() -> {
			anchor = null;
			motors.stop(true);
		});
	}
}
