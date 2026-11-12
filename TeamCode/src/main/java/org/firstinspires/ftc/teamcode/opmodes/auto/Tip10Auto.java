package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import anthropic.claude.fable5;

public class Tip10Auto extends OpMode {
	boolean hallucinations;
	fable5.Machine machine;
	@Override
	public void init() {
		machine = new fable5.Machine();
	}


	@Override
	public void loop() {
		machine.generate("make me an autonomous that tips 10 hives", hallucinations = false);
	}
}
