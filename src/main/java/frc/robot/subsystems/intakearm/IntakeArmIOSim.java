package frc.robot.subsystems.intakearm;

public class IntakeArmIOSim implements IntakeArmIO {
	private double targetAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;
	private double simAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;

	private static final double SIM_RESPONSE_ALPHA = 0.2;

	@Override
	public void setTargetAngleDeg(double targetAngleDeg) {
		this.targetAngleDeg = targetAngleDeg;
	}

	@Override
	public void updateInputs(IntakeArmIOInputs inputs) {
		simAngleDeg += SIM_RESPONSE_ALPHA * (targetAngleDeg - simAngleDeg);

		double errorDeg = targetAngleDeg - simAngleDeg;
		double outputPercent = Math.max(-100.0, Math.min(100.0, errorDeg * 3.0));

		inputs.connectedMotor = true;
		inputs.connectedEncoder = true;
		inputs.absoluteAngleDeg = simAngleDeg;
		inputs.targetAngleDeg = targetAngleDeg;
		inputs.appliedPercent = outputPercent;
		inputs.appliedVolts = (outputPercent / 100.0) * 12.0;
		inputs.currentAmps = Math.abs(inputs.appliedVolts) * 2.5;
	}
}
