package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Leave Auto (Soft Stop)")
public class LeaveAutoSoftStop extends LinearOpMode {
	private Robot robot;

	private DcMotorEx frontLeft, backLeft, frontRight, backRight;

	private static double power = 0.5;

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

		setMotorPowers(power, power, power, power);
		sleep(200);
	}


	public static DcMotorEx motorInit(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
		DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
		motor.setDirection(direction);
		motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		return motor;
	}

	public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
		frontLeft.setPower(frontLeftPower);
		backLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}

}