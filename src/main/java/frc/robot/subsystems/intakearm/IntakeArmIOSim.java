package frc.robot.subsystems.intakearm;

public class IntakeArmIOSim implements IntakeArmIO {
	private double targetAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;
	private double simAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;
	private double manualOutputPercent = 0.0;

	private static final double SIM_RESPONSE_ALPHA = 0.2;

	@Override
	public void setTargetAngleDeg(double targetAngleDeg) {
		this.targetAngleDeg = targetAngleDeg;
	}

	@Override
	public void updateInputs(IntakeArmIOInputs inputs) {
		if (Math.abs(manualOutputPercent) > 0.01) {
			simAngleDeg += manualOutputPercent * 0.1;
		} else {
			simAngleDeg += SIM_RESPONSE_ALPHA * (targetAngleDeg - simAngleDeg);
		}

		double errorDeg = targetAngleDeg - simAngleDeg;
		double outputPercent = Math.max(-100.0, Math.min(100.0, errorDeg * 3.0));

		inputs.connectedMotor = true;
		inputs.connectedEncoder = true;
		inputs.encoderAbsoluteRot = simAngleDeg / 360.0;
		inputs.encoderAbsoluteDeg = simAngleDeg;
		inputs.absoluteAngleDeg = simAngleDeg;
		inputs.targetAngleDeg = targetAngleDeg;
		inputs.appliedPercent = outputPercent;
		inputs.appliedVolts = (outputPercent / 100.0) * 12.0;
		inputs.currentAmps = Math.abs(inputs.appliedVolts) * 2.5;
	}

	@Override
	public void setManualOutputPercent(double percent) {
		manualOutputPercent = Math.max(-100.0, Math.min(100.0, percent));
	}

	@Override
	public void stop() {
		// Stop manual motion and freeze the target at the current simulated angle.
		manualOutputPercent = 0.0;
		targetAngleDeg = simAngleDeg;
	}
}
