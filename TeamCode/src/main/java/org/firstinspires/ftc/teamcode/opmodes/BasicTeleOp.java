package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.InitMenu;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.ShotCalculatorManualCloseFar;

@TeleOp(name = "Basic TeleOp")
public class BasicTeleOp extends OpMode {
	private Robot robot;

	private DcMotorEx frontLeft, backLeft, frontRight, backRight, intake, transfer, flywheel;

	private int rpm = 50;
	private boolean running = false;

	private double[] powers = new double[4];


	@Override
	public void init() {
		robot = new Robot(this, true);

		frontLeft = motorInit(hardwareMap, "frontLeft", DcMotorSimple.Direction.REVERSE);
		backLeft = motorInit(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE);
		frontRight = motorInit(hardwareMap, "frontRight", DcMotorSimple.Direction.FORWARD);
		backRight = motorInit(hardwareMap, "backRight", DcMotorSimple.Direction.FORWARD);
		intake = motorInit(hardwareMap, "intake", DcMotorSimple.Direction.FORWARD);
		transfer = motorInit(hardwareMap, "transfer", DcMotorSimple.Direction.FORWARD);
		flywheel = motorInit(hardwareMap, "flywheel", DcMotorSimple.Direction.FORWARD);

		frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void loop() {
		mecanumDrive(
				gamepad1.left_stick_y,
				-gamepad1.left_stick_x,
				gamepad1.right_stick_x,
				false
		);

		if (gamepad2.right_trigger > 0.1) {
			intake.setPower(-gamepad2.right_trigger);
		} else if (gamepad2.right_bumper) {
			intake.setPower(0.5);
		} else {
			intake.setPower(0);
		}


		if (gamepad2.leftBumperWasPressed()) {
			running = !running;
			if (running) {
				flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			} else {
				flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			}
		}

		if (gamepad2.dpadUpWasPressed()) {
			rpm += 10;
		} else if (gamepad2.dpadDownWasPressed()) {
			rpm -= 10;
		}

		if (running) {
			flywheel.setVelocity((double) -rpm / 60, AngleUnit.RADIANS);
		} else {
			flywheel.setPower(0);
		}

		if (gamepad1.left_trigger > 0.1) {
			transfer.setPower(1);
		} else if (gamepad1.left_bumper) {
			transfer.setPower(-1);
		} else {
			transfer.setPower(0);
		}


		telemetry.addData("Target RPM", rpm);
		telemetry.addData("Actual RPM", -flywheel.getVelocity(AngleUnit.RADIANS) * 60);
		telemetry.addData("Flywheel on", running);

		telemetry.update();
	}


	@Override
	public void stop() {
		robot.stop();
	}

	public static DcMotorEx motorInit(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
		DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
		motor.setDirection(direction);
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		return motor;
	}

	public void mecanumDrive(double straight, double strafe, double turn, boolean fineControl) {
		// calculate powers
		powers[0] = straight + strafe + turn; // front left power
		powers[1] = straight - strafe + turn; // back left power
		powers[2] = straight - strafe - turn; // front right power
		powers[3] = straight + strafe - turn; // back right power

		// powers array is updated inside this method
		normalizePowers(powers);

		for (int i = 0; i < 4; i++) {
			if (fineControl) {
				powers[i] *= 0.5;
			}
		}

		setMotorPowers(powers);
	}

	public void setMotorPowers(double[] powers) {
		if (powers.length == 4) {
			setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
		} else {
			throw new IllegalArgumentException("Powers array must have exactly 4 elements");
		}
	}

	public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
		frontLeft.setPower(frontLeftPower);
		backLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}

}