package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
	public static final String DRIVETRAIN_CAN_BUS = "Drivetrain";
	public static final String SUBSYSTEMS_CAN_BUS = "Subsystems";

	public static final class LedTestConstants {
		public static final boolean ENABLED = true;
		public static final int PWM_PORT = 0;
		public static final int LED_COUNT = 60;
		public static final int RED = 40;
		public static final int BLUE = 40;
		public static final int GREEN = 40;
	}
	

	public static enum Mode {
		
		REAL,

		SIM,

		REPLAY
	}

	public final class OperatorConstants {
		public static final int OPERATOR_CONTROLLER_PORT = 1;
	}

}
