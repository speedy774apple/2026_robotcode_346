package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public boolean connected2Inch1 = false;
		public boolean connected2Inch2 = false;
		public boolean connected3Inch1 = false;
		public boolean connected3Inch2 = false;
		public boolean connectedNeo550 = false;
		public boolean connectedRoller = false;

		public double velocity2Inch1Rps = 0.0;
		public double velocity2Inch2Rps = 0.0;
		public double velocity3Inch1Rps = 0.0;
		public double velocity3Inch2Rps = 0.0;

		public double appliedVolts2Inch1 = 0.0;
		public double appliedVolts2Inch2 = 0.0;
		public double appliedVolts3Inch1 = 0.0;
		public double appliedVolts3Inch2 = 0.0;
		public double appliedVoltsRoller = 0.0;

		public double currentAmps2Inch1 = 0.0;
		public double currentAmps2Inch2 = 0.0;
		public double currentAmps3Inch1 = 0.0;
		public double currentAmps3Inch2 = 0.0;
		public double currentAmpsRoller = 0.0;

		public double neoAppliedPercent = 0.0;
		public double rollerAppliedPercent = 0.0;
	}

	public default void updateInputs(ShooterIOInputs inputs) {
	}

	public default void setTargets(double twoInchRpm, double threeInchRpm, double neoPercent, double rollerPercent) {
	}

	public default void stop() {
	}
}
