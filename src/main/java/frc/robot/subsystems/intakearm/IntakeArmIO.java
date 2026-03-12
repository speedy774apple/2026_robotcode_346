package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
	@AutoLog
	public static class IntakeArmIOInputs {
		public boolean connectedMotor = false;
		public boolean connectedEncoder = false;

		public double encoderAbsoluteRot = 0.0;
		public double encoderAbsoluteDeg = 0.0;
		public double absoluteAngleDeg = 0.0;
		public double targetAngleDeg = 0.0;
		public double appliedVolts = 0.0;
		public double currentAmps = 0.0;
		public double appliedPercent = 0.0;
	}

	public default void updateInputs(IntakeArmIOInputs inputs) {
	}

	public default void setTargetAngleDeg(double targetAngleDeg) {
	}

	public default void stop() {
	}

	public default void setManualOutputPercent(double percent) {
	}
}
