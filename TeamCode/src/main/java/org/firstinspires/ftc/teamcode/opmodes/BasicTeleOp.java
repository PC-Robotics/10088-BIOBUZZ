package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.InitMenu;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.ShotCalculatorManualCloseFar;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide.LinearSlide;

@TeleOp(name = "Basic TeleOp")
public class BasicTeleOp extends OpMode {
	private Robot robot;
	private InitMenu initMenu;


	@Override
	public void init() {
		robot = new Robot(this, true);
		initMenu = new InitMenu(this);
	}


	@Override
	public void init_loop() {
		initMenu.update();
	}


	@Override
	public void start() {
		robot.setAlliance(initMenu.getAlliance());
		robot.slide.start();
	}


	@Override
	public void loop() {
		robot.follower.manual(
				-gamepad1.left_stick_y,
				gamepad1.left_stick_x,
				gamepad1.right_stick_x
		);

		intakeControl();
		flywheelControl();
		linearSlideControl();

		robot.periodic();
	}


	@Override
	public void stop() {
		robot.stop();
	}


	private void intakeControl() {
		if (gamepad1.aWasPressed()) {
			robot.intake.toggle().schedule();
		}
		if (gamepad1.bWasPressed()) {
			robot.intake.outtake().schedule();
		}
	}


	private void flywheelControl() {
		if (gamepad1.rightBumperWasPressed()) {
			robot.flywheel.toggleSpin().schedule();
		}
		if (gamepad1.dpadUpWasPressed()) {
			robot.flywheel.setManualPreset(ShotCalculatorManualCloseFar.ShotPreset.FAR);
		}
		if (gamepad1.dpadDownWasPressed()) {
			robot.flywheel.setManualPreset(ShotCalculatorManualCloseFar.ShotPreset.CLOSE);
		}
	}
}
