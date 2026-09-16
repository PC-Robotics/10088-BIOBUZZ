package org.firstinspires.ftc.teamcode.subsystems.gate;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.List;
import java.util.Locale;

public class Gate implements Subsystem {
	public enum State {
		OPEN, // meaning balls can pass through
		CLOSED
	}


	public ServoImplEx servo;
	private State gateState = State.CLOSED;

	private double openPosition = 0.7;
	private double closedPosition = 0.3;


	public Gate(OpMode opMode) {
		servo = opMode.hardwareMap.get(ServoImplEx.class, "gate");
		servo.setDirection(ServoImplEx.Direction.FORWARD);
	}


	public Command open() {
		return instant(() -> {
			servo.setPosition(openPosition);
			gateState = State.OPEN;
		}).requiring(servo);
	}


	public Command close() {
		return instant(() -> {
			servo.setPosition(closedPosition);
			gateState = State.CLOSED;
		}).requiring(servo);
	}


	public Command toggle() {
		return conditional(() -> gateState == State.OPEN, close(), open());
	}


	@Override
	public Command stop() {
		return close();
	}


	@Override
	public List<String> getSimpleTelemetry() {
		return List.of(
				"Gate State: " + gateState
		);
	}


	@Override
	public List<String> getDetailedTelemetry() {
		return List.of(
				"Gate State: " + gateState,
				"Servo Position: " + String.format(Locale.US, "%.2f", servo.getPosition())
		);
	}
}
