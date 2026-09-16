package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelShooter;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

public class Robot {
	private OpMode myOpMode;   // gain access to methods in the calling OpMode (iterative or linear).
	public TelemetryManager telemetry;

	// public DriveBase driveBase;
	public Intake intake;
	public FlywheelShooter flywheel;
	public LinearSlide slide;
	public Gate gate;
	private Subsystem[] subsystems;

	// every subsystem, driven uniformly at startup (stop + periodic) and shutdown (stop).
	// add new subsystems here so they get initialized and turned off automatically.
	private Subsystem[] subsystems;

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

	private static final double FIELD_MIRROR_LINE = 72.0;

	private static PoseFactory poses = PoseFactory.radians();

	// static variables are saved between auto and teleop so this variable helps us do that
	public static Pose scorePose; // legacy from decode

	static { // run once at init
		buildPoses();
	}

	private static void buildPoses() {
		scorePose = poses.of(56, 18, Math.toRadians(315));
	}


	private FollowerLog followerLog;


	public Robot(OpMode opMode, boolean isRobotCentric) {
		this.myOpMode = opMode;
		this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
		// drivetrain = new DriveTrain(myOpMode);
		intake = new Intake(myOpMode);
		gate = new Gate(myOpMode);
		flywheel = new FlywheelShooter(myOpMode, ShotCalculatorMode.MANUAL_CLOSE_FAR);
		slide = new LinearSlide(myOpMode);
		subsystems = new Subsystem[]{intake, gate, flywheel, slide};
		follower = Constants.create(myOpMode.hardwareMap).withLogger(log -> this.followerLog = log);

		hubs = myOpMode.hardwareMap.getAll(LynxModule.class);
		for (LynxModule h : hubs) {
			h.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		loop = new Timer();
		loop.reset();

		Scheduler.reset();
		for (Subsystem s : subsystems) {
			Scheduler.schedule(s.stop());
		}
		for (Subsystem s : subsystems) {
			Scheduler.schedule(s.periodic());
		}

		currentPose = null;
		this.isRobotCentric = isRobotCentric;
	}


	public void periodic() {
		// for loop timing
		loops++;
		if (loops == 10) {
			double now = loop.milliseconds();
			loopTime = (now - lastLoopTime) / 10;
			lastLoopTime = now;
			loops = 0;
		}

		follower.update();
		currentPose = follower.pose();

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
		if (followerLog != null) {
			lines.add("Follow State: " + followerLog.followState());
		}

		// get telemetry from opmode
		for (Supplier<List<String>> source : telemetrySources) {
			lines.addAll(source.get());
		}

		// get telemetry from subsystems
		lines.addAll(intake.getSimpleTelemetry());
		lines.addAll(flywheel.getSimpleTelemetry());
		lines.addAll(slide.getSimpleTelemetry());
		lines.addAll(gate.getSimpleTelemetry());

		telemetry.debug(lines.toArray(new String[0]));
		telemetry.update(myOpMode.telemetry);
	}


	public void addTelemetrySource(Supplier<List<String>> source) {
		telemetrySources.add(source);
	}


	public void stop() {
		endPose = follower.pose();

		Scheduler.reset();
		for (Subsystem s : subsystems) {
			Scheduler.schedule(s.stop());
		}
		Scheduler.execute();
	}


	public void setAlliance(Alliance alliance) {
		if (Robot.alliance != alliance) {
			poses = poses.mirrorX(FIELD_MIRROR_LINE);
			buildPoses();
		}

		Robot.alliance = alliance;
	}


	public FollowerLog getFollowerLog() {
		return followerLog;
	}


	public double getLoopTime() { // ms
		return loopTime;
	}
}