package frc.robot.subsystems.intakearm;

public final class IntakeArmConstants {
	private IntakeArmConstants() {
	}

	public static final int ARM_MOTOR_ID = 57;
	public static final int ARM_CANCODER_ID = 58;

	
	public static final double ARM_UP_REFERENCE_ROT = -0.279785;
	
	public static final boolean ARM_CANCODER_CCW_POSITIVE = true;

	public static final double ARM_UP_ANGLE_DEG = 0.0;

	public static final double ARM_DOWN_ANGLE_DEG = -76.38;

	
	public static final double ARM_DIRECTION = 1.0;

	// Tuning mode: POV controls manual open loop, arm is backdrivable.
	public static final boolean ARM_TUNING_MODE = true;
	// Manual output percent for POV up/down in tuning mode.
	public static final double ARM_MANUAL_OUTPUT_PERCENT = 10.0;
	// Soft limit buffer to prevent pushing into hard stops.
	public static final double ARM_SOFT_LIMIT_BUFFER_DEG = 0.5;

	
	public static final double ARM_KP = 0.03;
	public static final double ARM_MAX_OUTPUT = 0.1;
	// Minimum output to overcome static friction when moving.
	public static final double ARM_MIN_MOVING_OUTPUT = 0.04;
	public static final double ARM_ANGLE_TOLERANCE_DEG = 0.25;

	public static final double ARM_MOTOR_SUPPLY_CURRENT_LIMIT_A = 35.0;
	public static final double ARM_MOTOR_STATOR_CURRENT_LIMIT_A = 70.0;
	public static final double ARM_JOG_STEP_DEG = 1.0;
}
