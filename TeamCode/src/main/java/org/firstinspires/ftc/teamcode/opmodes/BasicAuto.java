package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Utility.normalizePowers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.InitMenu;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.ShotCalculatorManualCloseFar;

@Autonomous(name = "Basic Auto")
public class BasicAuto extends LinearOpMode {
	private Robot robot;

	private DcMotorEx frontLeft, backLeft, frontRight, backRight;

	private double[] powers = new double[4];


	@Override
	public void runOpMode() {
		robot = new Robot(this, true);

		frontLeft = motorInit(hardwareMap, "frontLeft", DcMotorSimple.Direction.REVERSE);
		backLeft = motorInit(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE);
		frontRight = motorInit(hardwareMap, "frontRight", DcMotorSimple.Direction.FORWARD);
		backRight = motorInit(hardwareMap, "backRight", DcMotorSimple.Direction.FORWARD);

		frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
		backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

		waitForStart();

		setMotorPowers(0.5, 0.5, 0.5, 0.5);
		sleep(500);
		frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}


	public static DcMotorEx motorInit(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
		DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
		motor.setDirection(direction);
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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