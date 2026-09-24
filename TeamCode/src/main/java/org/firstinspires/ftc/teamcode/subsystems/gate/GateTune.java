package org.firstinspires.ftc.teamcode.subsystems.gate;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp(name = "Gate Tune", group = "test")
public class GateTune extends OpMode {
	private Gate gate;

	private double tunePosition = 0.5;
	private static final double FINE_STEP = 0.01;
	private static final double COARSE_STEP = 0.05;


	@Override
	public void init() {
		gate = new Gate(this);

		Scheduler.reset();
		Scheduler.schedule(gate.stop());
	}


	@Override
	public void loop() {
		if (gamepad1.aWasPressed()) {
			gate.open().schedule();
		}
		if (gamepad1.bWasPressed()) {
			gate.close().schedule();
		}
		if (gamepad1.xWasPressed()) {
			gate.toggle().schedule();
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
			gate.servo.setPosition(tunePosition);
		}

		Scheduler.execute();

		telemetry.addLine("Gate Tune");
		telemetry.addLine("A = open | B = close | X = toggle");
		;
		telemetry.addLine("dpad = 0.01 | bumpers = 0.05");
		for (String line : gate.getDetailedTelemetry()) {
			telemetry.addLine(line);
		}
		telemetry.addData("Tune Position", String.format(Locale.US, "%.2f", tunePosition));
		telemetry.update();
	}


	@Override
	public void stop() {
		Scheduler.reset();
		Scheduler.schedule(gate.stop());
		Scheduler.execute();
	}
}