package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Locale;


// modular and generalized intake subsystem with jam detection and clearing, and item detection
@Configurable
public class Intake {
	public enum State {
		STOPPED,
		HOLDING,
		INTAKING,
		OUTTAKING
	}


	public DcMotorEx motor;
	private final DistanceSensor distanceSensor;

	// current mode label, set by the mode commands; used for detection gating + telemetry
	private State state = State.STOPPED;

	// powers are inverted in code
	private double holdingPower = 0.05;
	private double intakingPower = 1.0;
	private double outtakingPower = 0.3;
	private double jamClearingPower = 0.7;

	// jamming
	private double jamCurrentThreshold = 5.0; // amps
	private double jamVelocityThreshold = 50.0; // rpm
	private double jamTimeThreshold = 250.0; // ms
	private double jamMinPower = 0.4; // in [0, 1]
	private double jamSpinUpDelay = 500.0; // ms

	// jam clearing
	private double jamClearDuration = 250.0; // ms
	private boolean autoJamClearingEnabled = true;

	private boolean jamCandidate = false;
	private boolean jammed = false;
	private boolean clearing = false;

	private final ElapsedTime jamTimer = new ElapsedTime();
	private final ElapsedTime intakeRunTimer = new ElapsedTime();

	// item detection
	private boolean hasItem = false;
	private boolean itemCandidate = false;
	private double itemDistanceThreshold = 5.0; // cm

	// item must be seen continuously for this long
	private double itemDetectionTime = 60.0; // ms
	private final ElapsedTime itemDetectionTimer = new ElapsedTime();

	// single polling timer
	private double distanceSensorFastPollInterval = 20.0;  // ms
	private double distanceSensorSlowPollInterval = 200.0; // ms
	private final ElapsedTime distanceSensorPollTimer = new ElapsedTime();

	public Intake(LinearOpMode opMode) {
		motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");
	}

	// driver commands
	// new schedule interrupts running command
	public Command intake() {
		return Command.build() // runs continuously until interrupted
				.setStart(() -> { // only runs on first loop
					state = State.INTAKING; // set state
					intakeRunTimer.reset(); // start the intake run timer for jam detection
					motor.setPower(intakingPower);
				}).setDone(() -> false) // hold motor until interrupted
				.requiring(motor);
	}

	public Command stop() {
		return instant(() -> { // only runs one loop
			state = State.STOPPED;
			motor.setPower(0.0);
		}).requiring(motor);
	}

	public Command hold() {
		return instant(() -> { // only runs one loop
			state = State.HOLDING;
			motor.setPower(-holdingPower);
		}).requiring(motor);
	}

	public Command outtake() {
		return instant(() -> { // only runs one loop
			state = State.OUTTAKING;
			motor.setPower(-outtakingPower);
		}).requiring(motor);
	}

	public Command toggleIntake() {
		return conditional(() -> state == State.INTAKING, stop(), intake());
	}


	/**
	 * For clearing jams. it runs reverseMotorForClear() and runs the waitMs command at the same time, and as soon as either finishes it continues with intake.
	 * If user interrupts clearJam(), end behavior is called for reverseMotorForClear() effectively stopping the command
	 */
	public Command clearJam() {
		return race(reverseMotorForClear(), waitMs(jamClearDuration)).then(intake());
	}

	// actually controls motor
	private Command reverseMotorForClear() {
		return Command.build() // continuous function
				.setStart(() -> { // on first run
					clearing = true;
					motor.setPower(-jamClearingPower);
				}).setDone(() -> false) // hold until interrupted
				.setEnd(endCondition -> clearing = false) // on end (OR INTERRUPT), clearing = false
				.requiring(motor);
	}

	// runs detections
	public Command periodic() {
		return infinite(() -> { // runs forever
			detectItem();
			detectJam();
			if (autoJamClearingEnabled && state == State.INTAKING && jammed && !clearing) { // if should clear...
				clearJam().schedule(); // first build clearJam and then schedule it
			}
		});
	}

	// code to detect jam based on state and motor power, current, and velocity
	private void detectJam() {
		boolean sus = // initial flag
				state == State.INTAKING
				&& !clearing
				&& intakeRunTimer.milliseconds() >= jamSpinUpDelay
				&& Math.abs(motor.getPower()) >= jamMinPower
				&& motor.getCurrent(CurrentUnit.AMPS) >= jamCurrentThreshold
				&& getMotorVelocityRPM(motor) <= jamVelocityThreshold;

		if (sus) {
			if (!jamCandidate) { // edge detector (dont spam timer reset)
				jamCandidate = true;
				jamTimer.reset();
			}

			jammed = jamTimer.milliseconds() >= jamTimeThreshold;
		} else {
			jamCandidate = false;
			jammed = false;
		}
	}

	// code to detect item based on distance sensor
	private void detectItem() {
		// dynamic polling because sensor reads are EXPENSIVE
		// if robot already detects item, then poll fast. If there is no item, then poll slow.
		double pollInterval = (state == State.INTAKING || hasItem) ? distanceSensorFastPollInterval : distanceSensorSlowPollInterval;

		if (distanceSensorPollTimer.milliseconds() < pollInterval) {
			return;
		}
		distanceSensorPollTimer.reset();

		boolean sus = distanceSensor.getDistance(DistanceUnit.CM) <= itemDistanceThreshold; // initial flag

		if (sus) {
			if (!itemCandidate) { // edge detector (dont spam timer reset)
				itemCandidate = true;
				itemDetectionTimer.reset();
			}

			hasItem = itemDetectionTimer.milliseconds() >= itemDetectionTime;
		} else {
			itemCandidate = false;
			hasItem = false;
		}
	}

	// telemetry for robot controller
	public List<String> getSimpleTelemetry() {
		return List.of(
				"Intake State: " + state,
				"Has Item: " + hasItem,
				"Jammed: " + jammed,
				"Clearing: " + clearing,
				"Auto Jam Clear: " + autoJamClearingEnabled,
				"Power: " + String.format(Locale.US, "%.2f", motor.getPower())
		);
	}

	// just spamming atp
	public List<String> getDetailedTelemetry() {
		return List.of(
				"Intake State: " + state,
				"Motor Power: " + String.format(Locale.US, "%.2f", motor.getPower()),
				"Motor Current (A): " + String.format(Locale.US, "%.2f", motor.getCurrent(CurrentUnit.AMPS)),
				"Motor Velocity (RPM): " + String.format(Locale.US, "%.2f", getMotorVelocityRPM(motor)),
				"Auto Jam Clearing Enabled: " + autoJamClearingEnabled,
				"Clearing: " + clearing,
				"Has Item: " + hasItem,
				"Item Candidate: " + itemCandidate,
				"Last Item Distance (cm): " + String.format(Locale.US, "%.2f", distanceSensor.getDistance(DistanceUnit.CM)),
				"Item Distance Threshold (cm): " + String.format(Locale.US, "%.2f", itemDistanceThreshold),
				"Item Detection Time (ms): " + String.format(Locale.US, "%.1f", itemDetectionTime),
				"Item Fast Poll Interval (ms): " + String.format(Locale.US, "%.1f", distanceSensorFastPollInterval),
				"Item Slow Poll Interval (ms): " + String.format(Locale.US, "%.1f", distanceSensorSlowPollInterval),
				"Item Detect Timer (ms): " + String.format(Locale.US, "%.1f", itemDetectionTimer.milliseconds()),
				"Item Poll Timer (ms): " + String.format(Locale.US, "%.1f", distanceSensorPollTimer.milliseconds()),
				"Jam Candidate: " + jamCandidate,
				"Jammed: " + jammed,
				"Jam Timer (ms): " + String.format(Locale.US, "%.1f", jamTimer.milliseconds()),
				"Jam Current Threshold: " + String.format(Locale.US, "%.2f", jamCurrentThreshold),
				"Jam Velocity Threshold (RPM): " + String.format(Locale.US, "%.2f", jamVelocityThreshold),
				"Jam Time Threshold (ms): " + String.format(Locale.US, "%.1f", jamTimeThreshold),
				"Jam Spin-Up Delay (ms): " + String.format(Locale.US, "%.1f", jamSpinUpDelay),
				"Intake Run Timer (ms): " + String.format(Locale.US, "%.1f", intakeRunTimer.milliseconds())
		);
	}


	public void setAutoJamClearingEnabled(boolean enabled) {
		this.autoJamClearingEnabled = enabled;
	}

	public boolean isAutoJamClearingEnabled() {
		return autoJamClearingEnabled;
	}

	public void setHoldingPower(double power) {
		this.holdingPower = Math.abs(clamp(power, -1.0, 1.0));
	}

	public void setIntakingPower(double power) {
		this.intakingPower = Math.abs(clamp(power, -1.0, 1.0));
	}

	public void setOuttakingPower(double power) {
		this.outtakingPower = Math.abs(clamp(power, -1.0, 1.0));
	}

	public State getState() {
		return state;
	}

	public double getMotorPower() {
		return motor.getPower();
	}

	public boolean isJammed() {
		return jammed;
	}

	public boolean isClearing() {
		return clearing;
	}

	public boolean hasItem() {
		return hasItem;
	}

	public double getItemDistance() {
		return distanceSensor.getDistance(DistanceUnit.CM);
	}
}
