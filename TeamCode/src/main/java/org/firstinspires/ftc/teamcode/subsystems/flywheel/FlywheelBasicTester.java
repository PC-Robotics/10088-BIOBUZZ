package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class FlywheelBasicTester extends OpMode {
	public enum Mode {
		POWER,
		CONTROLLER
	}

	public static Mode mode = Mode.POWER;

	public DcMotorEx motor;
	public ServoImplEx hood;

	public static double power = 0;
	public static int targetRPM = 0;

	private FlywheelController controller = new FlywheelController();

	public static double hoodZeroPosition = 0.5;
	public static double hoodPosition = 0.0;

	private TelemetryManager panelsTelemetry;


	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


		motor = hardwareMap.get(DcMotorEx.class, "flywheel");
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		hood = hardwareMap.get(ServoImplEx.class, "hood");
		hood.setPosition(hoodZeroPosition);
	}

	@Override
	public void loop() {
		if (gamepad1.rightBumperWasPressed()) {
			mode = Mode.CONTROLLER;
		} else if (gamepad1.leftBumperWasPressed()) {
			mode = Mode.POWER;
		}

		double measuredRPM = motor.getVelocity(AngleUnit.RADIANS) * (60.0 / (2.0 * Math.PI));
		if (mode == Mode.POWER) {
			motor.setPower(power);
		} else if (mode == Mode.CONTROLLER) {
			controller.setTarget(targetRPM);
			motor.setPower(controller.update(measuredRPM));
		}

		hood.setPosition(hoodZeroPosition + hoodPosition);
		double angle = hoodPosition * 1800 / 6.1765; // 1800 degrees from 0 to 1, 17:105 gear ratio.

		panelsTelemetry.addData("Measured RPM", measuredRPM);
		panelsTelemetry.addData("Target RPM", targetRPM);
		panelsTelemetry.addData("Commanded Power", motor.getPower());
		panelsTelemetry.addData("Mode", mode);
		panelsTelemetry.addData("Hood Position", hoodPosition);
		panelsTelemetry.addData("Hood Angle", angle);
		panelsTelemetry.addData("Current", motor.getCurrent(CurrentUnit.AMPS));
	}
}
