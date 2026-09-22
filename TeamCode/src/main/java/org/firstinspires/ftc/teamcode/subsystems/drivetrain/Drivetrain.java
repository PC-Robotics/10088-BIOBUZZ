package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Drivetrain implements Subsystem {

	public Drivetrain(OpMode opMode) {
		// TODO - copy RevAmped codebase they do some pretty cool shit
	}


	// placeholder
	@Override
	public Command stop() {
		return instant(() -> {
		});
	}
}
