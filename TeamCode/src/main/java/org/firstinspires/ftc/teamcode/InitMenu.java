package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class InitMenu {
	private final OpMode opMode;
	private final TelemetryManager panels;

	private Alliance alliance = Alliance.BLUE; // alliance selection
	private double delaySeconds = 0; // delay before running program (for team play)


	public InitMenu(OpMode opMode) {
		this.opMode = opMode;
		this.panels = PanelsTelemetry.INSTANCE.getTelemetry();
		opMode.gamepad1.rumble(500); // reminder to set up the match before starting
	}


	public void update() {
		Gamepad g = opMode.gamepad1;

		if (g.leftBumperWasPressed()) {
			alliance = Alliance.BLUE;
		} else if (g.rightBumperWasPressed()) {
			alliance = Alliance.RED;
		}

		if (g.dpadUpWasPressed()) {
			delaySeconds += 0.5;
		} else if (g.dpadDownWasPressed()) {
			delaySeconds = Math.max(0, delaySeconds - 0.5);
		}

		panels.debug(
				"===== INIT MENU =====",
				"Alliance (bumpers L/R): " + alliance,
				"Delay (dpad up/down): " + delaySeconds + "s"
		);
		panels.update(opMode.telemetry);
	}


	public Alliance getAlliance() {
		return alliance;
	}


	public double getDelaySeconds() {
		return delaySeconds;
	}
}
