package frc.robot.subsystems.shooter;

public final class ShooterConstants {
	private ShooterConstants() {
	}

	
	public static final int TALON_2_INCH_1_ID = 40;
	public static final int TALON_2_INCH_2_ID = 41;
	public static final int TALON_3_INCH_1_ID = 42;
	public static final int TALON_3_INCH_2_ID = 43;
	public static final int NEO_550_ID = 44;
	public static final int ROLLER_ID = 46;

	
	public static final double TALON_2_INCH_TARGET_RPM = 6000.0;
	public static final double TALON_3_INCH_TARGET_RPM = 6000.0;
	public static final double NEO_550_SPEED_PERCENT = 75.0;
	public static final double ROLLER_SPEED_PERCENT = 65.0;
	public static final double SHOOT_DELAY_SECONDS = 1.0;

	
	public static final double TALON_2_INCH_1_DIR = -1.0;
	public static final double TALON_2_INCH_2_DIR = 1.0;
	public static final double TALON_3_INCH_1_DIR = 1.0;
	public static final double TALON_3_INCH_2_DIR = -1.0;
	public static final double ROLLER_DIR = 1.0;

	// Velocity control gains
	public static final double SHOOTER_KP = 0.32;
	public static final double SHOOTER_KI = 0.015;
	public static final double SHOOTER_KD = 0.001;
	public static final double SHOOTER_KV = 0.105;
	public static final double SHOOTER_KS = 0.10;

	// Limits / tolerances
	public static final double SPEED_TOLERANCE_RPS = 5.0;
	public static final double SHOOTER_SUPPLY_CURRENT_LIMIT_A = 80.0;
	public static final double SHOOTER_STATOR_CURRENT_LIMIT_A = 160.0;
}
