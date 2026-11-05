package org.firstinspires.ftc.teamcode.subsystems.flowerIntake;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp(name = "Flower Intake Tune", group = "test")
public class FlowerIntakeTune extends OpMode {
	private FlowerIntake flowerIntake;

	private double tunePosition = 0.5;
	private static final double FINE_STEP = 0.01;
	private static final double COARSE_STEP = 0.05;


	@Override
	public void init() {
		flowerIntake = new FlowerIntake(this);

		Scheduler.reset();
		Scheduler.schedule(flowerIntake.stop());
	}


	@Override
	public void loop() {
		if (gamepad1.aWasPressed()) {
			flowerIntake.open().schedule();
		}
		if (gamepad1.bWasPressed()) {
			flowerIntake.close().schedule();
		}
		if (gamepad1.xWasPressed()) {
			flowerIntake.toggle().schedule();
		}

		boolean tuned = false;
		if (gamepad1.dpadUpWasPressed()) {
			tunePosition += FINE_STEP;
			tuned = true;
		}
		if (gamepad1.dpadDownWasPressed()) {
			tunePosition -= FINE_STEP;
			tuned = true;
		}
		if (gamepad1.rightBumperWasPressed()) {
			tunePosition += COARSE_STEP;
			tuned = true;
		}
		if (gamepad1.leftBumperWasPressed()) {
			tunePosition -= COARSE_STEP;
			tuned = true;
		}
		if (tuned) {
			tunePosition = Math.max(0.0, Math.min(1.0, tunePosition));
			flowerIntake.servo.setPosition(tunePosition);
		}

		Scheduler.execute();

		telemetry.addLine("Flower Intake Tune");
		telemetry.addLine("A = open | B = close | X = toggle");
		telemetry.addLine("dpad = 0.01 | bumpers = 0.05");
		for (String line : flowerIntake.getDetailedTelemetry()) {
			telemetry.addLine(line);
		}
		telemetry.addData("Tune Position", String.format(Locale.US, "%.2f", tunePosition));
		telemetry.update();
	}


	@Override
	public void stop() {
		Scheduler.reset();
		Scheduler.schedule(flowerIntake.stop());
		Scheduler.execute();
	}
}
