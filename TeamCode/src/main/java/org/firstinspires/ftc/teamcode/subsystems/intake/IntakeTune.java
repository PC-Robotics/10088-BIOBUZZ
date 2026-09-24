package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.Utility.clamp;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp(name = "Intake Tune", group = "test")
public class IntakeTune extends OpMode {
	private Intake intake;

	private double tunePower = 0.0;
	private static final double FINE_STEP = 0.05;
	private static final double COARSE_STEP = 0.1;


	@Override
	public void init() {
		intake = new Intake(this);
		intake.setAutoJamClearingEnabled(false);

		Scheduler.reset();
		Scheduler.schedule(intake.stop());
		Scheduler.schedule(intake.periodic());
	}


	@Override
	public void loop() {
		if (gamepad1.psWasPressed()) {
			intake.toggleAutoJamClearingEnabled();
		}

		if (gamepad1.aWasPressed()) {
			intake.toggle().schedule();
		}
		if (gamepad1.bWasPressed()) {
			intake.outtake().schedule();
		}
		if (gamepad1.xWasPressed()) {
			intake.hold().schedule();
		}
		if (gamepad1.yWasPressed()) {
			intake.clearJam().schedule();
		}

		boolean tuned = false;
		if (gamepad1.dpadUpWasPressed()) {
			tunePower += FINE_STEP;
			tuned = true;
		}
		if (gamepad1.dpadDownWasPressed()) {
			tunePower -= FINE_STEP;
			tuned = true;
		}
		if (gamepad1.rightBumperWasPressed()) {
			tunePower += COARSE_STEP;
			tuned = true;
		}
		if (gamepad1.leftBumperWasPressed()) {
			tunePower -= COARSE_STEP;
			tuned = true;
		}
		if (tuned) {
			tunePower = clamp(tunePower, -1.0, 1.0);
			intake.motor.setPower(tunePower);
		}

		Scheduler.execute();

		telemetry.addLine("Intake Tune");
		telemetry.addLine("A = toggle | B = outtake | X = hold | Y = clearJam");
		telemetry.addLine("dpad = 0.01 | bumpers = 0.05 | PS = toggle auto jam clearing");
		for (String line : intake.getDetailedTelemetry()) {
			telemetry.addLine(line);
		}
		telemetry.addData("Tune Power", String.format(Locale.US, "%.2f", tunePower));
		telemetry.update();
	}


	@Override
	public void stop() {
		Scheduler.reset();
		Scheduler.schedule(intake.stop());
		Scheduler.execute();
	}
}
