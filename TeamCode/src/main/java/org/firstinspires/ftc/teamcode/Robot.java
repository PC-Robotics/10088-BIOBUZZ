package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelShooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

public class Robot {
	private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
	public TelemetryManager telemetry;

	// public DriveBase driveBase;
	public Intake intake;
	public FlywheelShooter flywheel;

	public boolean isRobotCentric = false;

	// pedro
	public Follower follower;
	public static Alliance alliance = Alliance.BLUE;

	private Timer loop;
	public int loops = 0;
	public double loopTime = 0, lastLoopTime = 0;
	private List<LynxModule> hubs;

	// overengineered telemetry
	private final List<Supplier<List<String>>> telemetrySources = new ArrayList<>();

	public Pose currentPose;
	public static Pose endPose;
	// static variables are saved between auto and teleop so this variable helps us do that
	public static Pose scorePose = new Pose(56, 18, Math.toRadians(315)); // legacy from decode

	// Define a constructor that allows the OpMode to pass a reference to itself.
	public Robot(LinearOpMode opMode, boolean isRobotCentric) {
		this.myOpMode = opMode;
		this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
		// drivetrain = new DriveTrain(myOpMode);
		intake = new Intake(myOpMode);
		flywheel = new FlywheelShooter(myOpMode, ShotCalculatorMode.MANUAL_CLOSE_FAR);
		follower = Constants.createFollower(myOpMode.hardwareMap);

		hubs = myOpMode.hardwareMap.getAll(LynxModule.class);
		for (LynxModule h : hubs) {
			h.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		loop = new Timer();
		loop.resetTimer();

		Scheduler.reset();
		Scheduler.schedule(
				intake.stop(), flywheel.stop(),
				// drivetrain.stop(),
				intake.periodic(), flywheel.periodic()
				// drivetrain.periodic()
		);

		currentPose = null;
		this.isRobotCentric = isRobotCentric;
	}


	public void periodic() {
		// for loop timing
		loops++;
		if (loops == 10) {
			double now = loop.getElapsedTime();
			loopTime = (now - lastLoopTime) / 10;
			lastLoopTime = now;
			loops = 0;
		}

		follower.update();
		currentPose = follower.getPose();

		flywheel.updateRobotPose(currentPose);
		flywheel.updateGoalPose(scorePose);

		Scheduler.execute();

		reloadTelemetry();
	}


	public void reloadTelemetry() {
		List<String> lines = new ArrayList<>();

		lines.add("Alliance: " + alliance);
		lines.add("Pose: " + currentPose);
		lines.add("Loop Time (ms): " + String.format(Locale.US, "%.2f", loopTime));

		// get telemetry from opmode
		for (Supplier<List<String>> source : telemetrySources) {
			lines.addAll(source.get());
		}

		// get telemetry from subsystems
		lines.addAll(intake.getSimpleTelemetry());
		lines.addAll(flywheel.getSimpleTelemetry());

		telemetry.debug(lines.toArray(new String[0]));
		telemetry.update(myOpMode.telemetry);
	}


	public void addTelemetrySource(Supplier<List<String>> source) {
		telemetrySources.add(source);
	}


	public void stop() {
		endPose = follower.getPose();
	}


	public void setAlliance(Alliance alliance) {
		if (Robot.alliance != alliance) {
			scorePose = scorePose.mirror();
		}

		Robot.alliance = alliance;
	}


	public double getLoopTime() { // ms
		return loopTime;
	}
}