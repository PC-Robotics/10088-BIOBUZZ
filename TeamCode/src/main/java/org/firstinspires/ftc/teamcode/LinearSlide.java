package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LinearSlideController.Position;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.List;
import java.util.Locale;


@Configurable
public class LinearSlide implements Subsystem {
	public enum ControlMode {
		BUTTON, // buttons map to preset positions
		MANUAL // buttons slowly move the slide up and down
	}


	public DcMotorEx motor;
	private final LinearSlideController controller = new LinearSlideController();

	private Position slidePosition = Position.FLOOR;
	private ControlMode controlMode = ControlMode.BUTTON;

	// TODO - tune
	public static double MANUAL_POWER = 0.0;


	public LinearSlide(OpMode opMode) {
		motor = opMode.hardwareMap.get(DcMotorEx.class, "linearSlide");
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
	}


	public void goToDropHeight() {
		slidePosition = Position.DROP;
	}


	public void goToGround() {
		slidePosition = Position.FLOOR;
	}


	public void toggleManualMode() {
		controlMode = (controlMode == ControlMode.MANUAL) ? ControlMode.BUTTON : ControlMode.MANUAL;
		start();
	}


	// manual move fallback
	public void manualMove(int direction) {
		motor.setPower(direction * MANUAL_POWER);
	}


	public void start() {
		if (controlMode == ControlMode.MANUAL) {
			motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // hold
		} else {
			motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		}
	}


	@Override
	public Command stop() {
		return instant(() -> {
			motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // hold
			motor.setPower(0.0);
		});
	}


	public Command reset() {
		return instant(() -> {
			motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
			motor.setPower(0.0);
			slidePosition = Position.FLOOR;
		});
	}


	@Override
	public Command periodic() {
		return infinite(() -> {
			if (controlMode == ControlMode.MANUAL) {
				return;
			}
			// if controlMode is button
			double power = controller.update(
					slidePosition,
					motor.getCurrentPosition(),
					motor.getVelocity(),
					() -> motor.getCurrent(CurrentUnit.AMPS)
			);
			motor.setPower(power);
		});
	}


	@Override
	public List<String> getSimpleTelemetry() {
		return List.of(
				"Slide Position: " + slidePosition,
				"Control Mode: " + controlMode,
				"Target (ticks): " + slidePosition.ticks,
				"Position (ticks): " + motor.getCurrentPosition(),
				"kG: " + String.format(Locale.US, "%.3f", controller.getKg())
		);
	}


	@Override
	public List<String> getDetailedTelemetry() {
		return getSimpleTelemetry();
	}


	public ControlMode getControlMode() {
		return controlMode;
	}
}
