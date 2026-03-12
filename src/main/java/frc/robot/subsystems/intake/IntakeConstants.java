package frc.robot.subsystems.intake;

public final class IntakeConstants {
	private IntakeConstants() {
	}

	
	public static final int INTAKE_MOTOR_ID = 56;

	public static final double INTAKE_SPEED_PERCENT = 75.0;
	public static final double INTAKE_DIRECTION = 1.0;
	// Time for open loop soft start (handled by motor controller)
	public static final double INTAKE_OPEN_LOOP_RAMP_SECONDS = 0.65;

	public static final double INTAKE_SUPPLY_CURRENT_LIMIT_A = 40.0;
	public static final double INTAKE_STATOR_CURRENT_LIMIT_A = 80.0;
}
