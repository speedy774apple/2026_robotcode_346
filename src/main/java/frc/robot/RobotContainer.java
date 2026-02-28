package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.commands.AkitDriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intakearm.IntakeArm;
import frc.robot.subsystems.intakearm.IntakeArmIO;
import frc.robot.subsystems.intakearm.IntakeArmIOReal;
import frc.robot.subsystems.intakearm.IntakeArmIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterAutoMap;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.*;

public class RobotContainer {
	private static final boolean DRIVE_ENABLED = true;

	private final Drive drive;
	private final VisionLocalizer vision;
	private final Shooter shooter;
	private final Intake intake;
	private final IntakeArm intakeArm;

	private final CommandXboxController controller = new CommandXboxController(0);

	private final LoggedDashboardChooser<Command> autoChooser;

	// Field-relative mode toggle (default: field-relative)
	private boolean useFieldRelative = true;

	// Drive controls inversion toggle
	private boolean controlsInverted = false;

	public RobotContainer() {
		// Create drive subsystem based on mode
		switch (Constants.currentMode) {
			case REAL:
				drive = new Drive(
						new GyroIOPigeon2(),
						new ModuleIOTalonFX(TunerConstants.FrontLeft),
						new ModuleIOTalonFX(TunerConstants.FrontRight),
						new ModuleIOTalonFX(TunerConstants.BackLeft),
						new ModuleIOTalonFX(TunerConstants.BackRight));
				vision = createVisionSystem();
				break;

			case SIM:
				drive = new Drive(
						new GyroIO() {},
						new ModuleIOSim(TunerConstants.FrontLeft),
						new ModuleIOSim(TunerConstants.FrontRight),
						new ModuleIOSim(TunerConstants.BackLeft),
						new ModuleIOSim(TunerConstants.BackRight));
				vision = createVisionSystem();
				break;

			default:
				drive = new Drive(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {});
				vision = new VisionLocalizer(drive::addVisionMeasurement, drive::getPose, new VisionIO() {});
		}

		ShooterIO shooterIO = switch (Constants.currentMode) {
			case REAL -> new ShooterIOReal();
			case SIM -> new ShooterIOSim();
			default -> new ShooterIO() {};
		};
		shooter = new Shooter(shooterIO);

		IntakeIO intakeIO = switch (Constants.currentMode) {
			case REAL -> new IntakeIOReal();
			case SIM -> new IntakeIOSim();
			default -> new IntakeIO() {};
		};
		intake = new Intake(intakeIO);

		IntakeArmIO intakeArmIO = switch (Constants.currentMode) {
			case REAL -> new IntakeArmIOReal();
			case SIM -> new IntakeArmIOSim();
			default -> new IntakeArmIO() {};
		};
		intakeArm = new IntakeArm(intakeArmIO);
		vision.setVisionConsumer(drive::addVisionMeasurement);

		autoChooser = new LoggedDashboardChooser<>("Auto Choices");

		configureButtonBindings();
	}

	/**
	 * Creates vision system with automatically detected cameras.
	 */
	private VisionLocalizer createVisionSystem() {
		if (Constants.currentMode == Constants.Mode.REAL) {
			return new VisionLocalizer(
					drive::addVisionMeasurement,
					drive::getPose,
					// FL camera commented out - not installed IRL
					// new VisionIOPhotonReal(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0]),
					new VisionIOPhotonReal(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1]),
					new VisionIOPhotonReal(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2]),
					new VisionIOPhotonReal(VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3]));
		} else {
			return new VisionLocalizer(
					drive::addVisionMeasurement,
					drive::getPose,
					// FL camera commented out - not installed IRL
					// new VisionIOPhotonSim(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0], drive::getPose),
					new VisionIOPhotonSim(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1], drive::getPose),
					new VisionIOPhotonSim(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2], drive::getPose),
					new VisionIOPhotonSim(VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3], drive::getPose));
		}
	}

	private static final Translation2d AIM_TARGET = new Translation2d(11.907, 4.030);

	private void configureButtonBindings() {
		if (DRIVE_ENABLED) {
			// Default command: field-relative drive; LT = aim at target (right stick disabled while aiming)
			drive.setDefaultCommand(
					AkitDriveCommands.joystickDriveWithAim(
							drive,
							() -> controlsInverted ? -controller.getLeftY() : controller.getLeftY(),
							() -> controlsInverted ? -controller.getLeftX() : controller.getLeftX(),
							() -> controlsInverted ? controller.getRightX() : -controller.getRightX(),
							() -> useFieldRelative,
							() -> controller.leftTrigger().getAsBoolean(),
							AIM_TARGET));
		} else {
			// Temporary drive disable for testing other mechanisms.
			drive.setDefaultCommand(Commands.run(drive::stop, drive));
		}

		// Toggle drive controls inversion on X button
		controller.x().onTrue(
				Commands.runOnce(
						() -> {
							controlsInverted = !controlsInverted;
							System.out.println("Drive controls: " + (controlsInverted ? "INVERTED" : "NORMAL"));
						}));

		// Toggle field-relative mode on B button
		controller.b().onTrue(
				Commands.runOnce(
						() -> {
							useFieldRelative = !useFieldRelative;
							System.out.println("Drive mode: " + (useFieldRelative ? "FIELD-RELATIVE" : "ROBOT-RELATIVE"));
						}));

		// Reset robot heading on A button press
		controller.a().onTrue(
				Commands.runOnce(
						() -> {
							drive.zeroGyro();
							System.out.println("Robot heading reset - current front is now forward");
						},
						drive)
						.ignoringDisable(true));

		// Zero gyro heading on START button press
		controller.start().onTrue(
				Commands.runOnce(
						() -> {
							drive.zeroGyro();
							System.out.println("Gyro zeroed - robot heading reset to zero");
						},
						drive)
						.ignoringDisable(true));

		// Reset pose (position and heading) on right bumper
		controller.rightBumper().onTrue(
				Commands.runOnce(
						() -> drive.setPose(
								new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
						drive)
						.ignoringDisable(true));

		// Shooter: hold Y for distance-based auto shoot, stop on release
		controller.y()
				.whileTrue(shooter.runAutoShoot(
						() -> ShooterAutoMap.getDistanceFeet(drive.getPose(), AIM_TARGET)))
				.onFalse(shooter.stopCoralIntake());

		// Intake: hold left bumper to run motor, stop on release
		controller.leftBumper()
				.whileTrue(intake.runIntake())
				.onFalse(intake.stopIntake());

		// Intake arm position control using absolute encoder
		// D-pad down: move arm to 72 deg down, D-pad up: move arm fully up
		controller.povDown().onTrue(intakeArm.moveDownCommand());
		controller.povUp().onTrue(intakeArm.moveUpCommand());
	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	public void teleopInit() {
		// Teleop initialization if needed
	}

	public Drive getDrive() {
		return drive;
	}

	public VisionLocalizer getVision() {
		return vision;
	}

	public Shooter getShooter() {
		return shooter;
	}

	public Intake getIntake() {
		return intake;
	}

	public IntakeArm getIntakeArm() {
		return intakeArm;
	}
}
