package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IntakeIO {
	private double appliedPercent = 0.0;

	@Override
	public void setPercent(double percent) {
		appliedPercent = MathUtil.clamp(percent, -100.0, 100.0) * IntakeConstants.INTAKE_DIRECTION;
	}

	@Override
	public void stop() {
		appliedPercent = 0.0;
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.connected = true;
		inputs.appliedPercent = appliedPercent;
		inputs.appliedVolts = (appliedPercent / 100.0) * 12.0;
		inputs.currentAmps = Math.abs(inputs.appliedVolts) * 2.0;
	}
}
