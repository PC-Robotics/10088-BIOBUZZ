package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;

import java.util.Collections;
import java.util.List;

// common surface for team subsystems in the Ivy command style: subsystems expose behavior as
// Commands the Scheduler runs, not imperative methods. Hardware is initialized in the constructor.
public interface Subsystem {
	// constructor should have param (OpMode opMode) and init hardware

	Command stop();

	default Command periodic() {
		return infinite(() -> {
		});
	}

	default List<String> getSimpleTelemetry() {
		return Collections.emptyList();
	}

	default List<String> getDetailedTelemetry() {
		return Collections.emptyList();
	}
}
