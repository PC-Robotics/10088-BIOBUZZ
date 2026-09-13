package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.shooting.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorDistance;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorManualCloseFar;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorMode;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorProportional;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotSolution;

import java.util.EnumMap;
import java.util.List;
import java.util.Locale;

// shooter class. It can hot-load different shot calculation methods if one fails.
@Configurable
public class FlywheelShooter {
	public enum State {
		STOPPED,
		SPINNING
	}


	public enum LedColor {
		OFF(0.0),
		GREEN(0.5),
		RED(0.29),
		YELLOW(0.388),
		BLUE(0.65),
		WHITE(1.0);

		private final double value;

		LedColor(double value) {
			this.value = value;
		}

		public double getValue() {
			return value;
		}
	}


	public DcMotorEx leftMotor;
	public DcMotorEx rightMotor;
	public ServoImplEx light;

	private final FlywheelController controller = new FlywheelController();

	private State flywheelState = State.STOPPED;

	private ShotCalculatorMode shotCalculatorMode;

	private double commandedPower = 0.0;
	private double targetRPM = 0.0;
	private LedColor commandedLedColor = LedColor.OFF;

	private Pose robotPose;
	private Pose goalPose;
	private ShotSolution currentShotSolution = new ShotSolution(0.0, 0.0, 0.0, false);

	private final EnumMap<ShotCalculatorMode, ShotCalculator> calculators = new EnumMap<>(ShotCalculatorMode.class);
	private final ShotCalculatorManualCloseFar manualCloseFarCalculator = new ShotCalculatorManualCloseFar();

	public FlywheelShooter(OpMode opMode, @NonNull ShotCalculatorMode shotCalculatorMode) {
		this.shotCalculatorMode = shotCalculatorMode;

		calculators.put(ShotCalculatorMode.MANUAL_CLOSE_FAR, manualCloseFarCalculator);
		calculators.put(ShotCalculatorMode.DISTANCE, new ShotCalculatorDistance());
		calculators.put(ShotCalculatorMode.PROPORTIONAL, new ShotCalculatorProportional());
		calculators.values().forEach(ShotCalculator::init);
		manualCloseFarCalculator.setPreset(ShotCalculatorManualCloseFar.ShotPreset.CLOSE);

		leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelleft");
		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelright");
		rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		light = opMode.hardwareMap.get(ServoImplEx.class, "light");
		light.setPosition(LedColor.OFF.getValue());
	}

	// driver commands
	// new schedule interrupts running command

	public Command spin() {
		return Command.build().setStart(() -> {
			flywheelState = State.SPINNING;
			controller.reset(measuredRPM());
		}).setExecute(() -> {
			controller.setTarget(targetRPM);
			commandedPower = controller.update(measuredRPM());
			leftMotor.setPower(commandedPower);
			rightMotor.setPower(commandedPower);

			if (!currentShotSolution.isValid()) {
				commandedLedColor = LedColor.YELLOW;
			} else {
				commandedLedColor = isReadyToShoot() ? LedColor.GREEN : LedColor.RED;
			}
			light.setPosition(commandedLedColor.getValue());
		}).setDone(() -> false).setEnd(endCondition -> stopAction()).requiring(leftMotor, rightMotor);
	}


	public Command stop() {
		return instant(this::stopAction).requiring(leftMotor, rightMotor);
	}


	private void stopAction() {
		flywheelState = State.STOPPED;
		commandedPower = 0.0;
		commandedLedColor = LedColor.OFF;
		leftMotor.setPower(0.0);
		rightMotor.setPower(0.0);
		light.setPosition(LedColor.OFF.getValue());
	}


	public Command toggleSpin() {
		return conditional(() -> flywheelState == State.SPINNING, stop(), spin());
	}

	// run every single loop
	public Command periodic() {
		return infinite(this::updateShotSolution);
	}

	private double measuredRPM() {
		return (getMotorVelocityRPM(leftMotor) + getMotorVelocityRPM(rightMotor)) * 0.5;
	}

	// controller-derived values (filtered RPM, error) are only meaningful while it's being updated
	private boolean controllerRunning() {
		return flywheelState == State.SPINNING;
	}

	private void updateShotSolution() {
		ShotCalculator active = calculators.getOrDefault(shotCalculatorMode, manualCloseFarCalculator);

		if (robotPose != null) {
			active.updateRobotPose(robotPose);
		}
		if (goalPose != null) {
			active.updateGoalPose(goalPose);
		}

		currentShotSolution = active.run();
		targetRPM = currentShotSolution.isValid() ? currentShotSolution.getTargetRPM() : 0.0;
	}

	public boolean isReadyToShoot() {
		return flywheelState == State.SPINNING
				&& currentShotSolution.isValid()
				&& controller.getState() == FlywheelController.State.READY;
	}

	// telemetry for robot controller
	public List<String> getSimpleTelemetry() {
		return List.of(
				"Flywheel State: " + flywheelState,
				"Calculator Mode: " + shotCalculatorMode,
				"Manual Preset: " + manualCloseFarCalculator.getPreset(),
				"Ready To Shoot: " + isReadyToShoot(),
				"Power: " + String.format(Locale.US, "%.2f", commandedPower)
		);
	}

	public List<String> getDetailedTelemetry() {
		return List.of(
				"Flywheel State: " + flywheelState,
				"Controller State: " + controller.getState(),
				"Calculator Mode: " + shotCalculatorMode,
				"Manual Preset: " + manualCloseFarCalculator.getPreset(),
				"Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
				"Current RPM: " + String.format(Locale.US, "%.2f", measuredRPM()),
				"Filtered RPM: " + String.format(Locale.US, "%.2f", controllerRunning() ? controller.getFilteredRPM() : Double.NaN),
				"Target RPM: " + String.format(Locale.US, "%.2f", targetRPM),
				"Error: " + String.format(Locale.US, "%.2f", controllerRunning() ? controller.getError() : Double.NaN),
				"Shot Valid: " + currentShotSolution.isValid(),
				"Shot Distance: " + String.format(Locale.US, "%.2f", currentShotSolution.getDistance()),
				"Ready To Shoot: " + isReadyToShoot(),
				"LED Color: " + commandedLedColor
		);
	}


	// getters and setters
	public void setManualPreset(@NonNull ShotCalculatorManualCloseFar.ShotPreset preset) {
		manualCloseFarCalculator.setPreset(preset);
	}

	public void setShotCalculatorMode(@NonNull ShotCalculatorMode shotCalculatorMode) {
		this.shotCalculatorMode = shotCalculatorMode;
	}

	public void updateRobotPose(Pose robotPose) {
		this.robotPose = robotPose;
	}

	public void updateGoalPose(Pose goalPose) {
		this.goalPose = goalPose;
	}
}
