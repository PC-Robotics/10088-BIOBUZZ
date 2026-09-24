package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import anthropic.claude.opus5;

public class Tip5Auto extends OpMode {
	boolean hallucinations;
	opus5.Machine machine;
	@Override
	public void init() {
		machine = new opus5.Machine();
	}


	@Override
	public void loop() {
		machine.generate("make me an auto that tips 10 hives", hallucinations = false);
	}
}
