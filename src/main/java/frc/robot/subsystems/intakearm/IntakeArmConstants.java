package frc.robot.subsystems.intakearm;

public final class IntakeArmConstants {
	private IntakeArmConstants() {
	}

	public static final int ARM_MOTOR_ID = 47;
	public static final int ARM_CANCODER_ID = 48;

	// 0 deg is arm fully up. Positive is down by default
	public static final double ARM_UP_ANGLE_DEG = 0.0;
	public static final double ARM_DOWN_ANGLE_DEG = 72.0;

	// Flip this to -1.0 the arm moves opposite direction.
	public static final double ARM_DIRECTION = 1.0;

	// Tune these on robot.
	public static final double ARM_KP = 0.03;
	public static final double ARM_MAX_OUTPUT = 0.45;
	public static final double ARM_ANGLE_TOLERANCE_DEG = 1.5;

	public static final double ARM_MOTOR_SUPPLY_CURRENT_LIMIT_A = 35.0;
	public static final double ARM_MOTOR_STATOR_CURRENT_LIMIT_A = 70.0;
}
