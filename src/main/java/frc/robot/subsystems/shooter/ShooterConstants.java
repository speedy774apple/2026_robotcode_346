package frc.robot.subsystems.shooter;

public final class ShooterConstants {
	private ShooterConstants() {
	}

	
	// Shooter: 51-55
	public static final int TALON_2_INCH_ID = 51;
	public static final int TALON_3_INCH_1_ID = 52;
	public static final int TALON_3_INCH_2_ID = 53;
	public static final int FEEDER_ROLLER_ID = 54;
	public static final int NEO_550_ID = 55;

	public static final double TALON_2_INCH_TARGET_RPM = 4260.0; 
	public static final double TALON_3_INCH_TARGET_RPM = 3120.0;
	public static final double NEO_550_SPEED_PERCENT = 100.0;
	public static final double ROLLER_SPEED_PERCENT = 100.0;
	public static final double SHOOT_DELAY_SECONDS = 0.5;

	public static final double TALON_2_INCH_DIR = 1.0;
	public static final double TALON_3_INCH_1_DIR = 1.0;
	public static final double TALON_3_INCH_2_DIR = -1.0;
	public static final double FEEDER_ROLLER_DIR = 1.0;
	// Time for feeder roller open-loop soft start (handled by motor controller)
	public static final double FEEDER_ROLLER_OPEN_LOOP_RAMP_SECONDS = 0.65;

	public static final double SHOOTER_KP = 0.32;
	public static final double SHOOTER_KI = 0.015;
	public static final double SHOOTER_KD = 0.001;
	public static final double SHOOTER_KV = 0.105;
	public static final double SHOOTER_KS = 0.10;

	public static final double SPEED_TOLERANCE_RPS = 5.0;
	public static final double SHOOTER_SUPPLY_CURRENT_LIMIT_A = 80.0;
	public static final double SHOOTER_STATOR_CURRENT_LIMIT_A = 160.0;
}
