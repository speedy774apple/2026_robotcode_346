package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	private double target2InchRpm = 0.0;
	private double target3InchRpm = 0.0;
	private double targetNeoPercent = 0.0;
	private double targetRollerPercent = 0.0;
	private double autoDistanceFeet = 0.0;
	private double autoThreeInchPercent = 0.0;
	private boolean enabled = false;

	public Shooter(ShooterIO io) {
		this.io = io;
	}

	public void setTargets(double twoInchRpm, double threeInchRpm, double neoPercent, double rollerPercent) {
		target2InchRpm = twoInchRpm;
		target3InchRpm = threeInchRpm;
		targetNeoPercent = neoPercent;
		targetRollerPercent = rollerPercent;
		enabled = true;
		io.setTargets(target2InchRpm, target3InchRpm, targetNeoPercent, targetRollerPercent);
	}

	public void stop() {
		enabled = false;
		target2InchRpm = 0.0;
		target3InchRpm = 0.0;
		targetNeoPercent = 0.0;
		targetRollerPercent = 0.0;
		io.stop();
	}

	private static double rpmToRps(double rpm) {
		return rpm / 60.0;
	}

	@AutoLogOutput(key = "Shooter/AtVelocitySetpoint")
	public boolean atVelocitySetpoint() {
		double target2Rps = rpmToRps(target2InchRpm);
		double target3Rps = rpmToRps(target3InchRpm);

		double twoInch1Error = Math.abs(inputs.velocity2Inch1Rps - (target2Rps * ShooterConstants.TALON_2_INCH_1_DIR));
		double twoInch2Error = Math.abs(inputs.velocity2Inch2Rps - (target2Rps * ShooterConstants.TALON_2_INCH_2_DIR));
		double threeInch1Error = Math.abs(inputs.velocity3Inch1Rps - (target3Rps * ShooterConstants.TALON_3_INCH_1_DIR));
		double threeInch2Error = Math.abs(inputs.velocity3Inch2Rps - (target3Rps * ShooterConstants.TALON_3_INCH_2_DIR));

		return twoInch1Error <= ShooterConstants.SPEED_TOLERANCE_RPS
				&& twoInch2Error <= ShooterConstants.SPEED_TOLERANCE_RPS
				&& threeInch1Error <= ShooterConstants.SPEED_TOLERANCE_RPS
				&& threeInch2Error <= ShooterConstants.SPEED_TOLERANCE_RPS;
	}

	public Command runShoot() {
		return Commands.sequence(
				Commands.runOnce(
						() -> setTargets(
								0.0,
								0.0,
								0.0,
								ShooterConstants.ROLLER_SPEED_PERCENT),
						this),
				Commands.waitSeconds(ShooterConstants.SHOOT_DELAY_SECONDS),
				Commands.run(
						() -> setTargets(
								ShooterConstants.TALON_2_INCH_TARGET_RPM,
								ShooterConstants.TALON_3_INCH_TARGET_RPM,
								ShooterConstants.NEO_550_SPEED_PERCENT,
								ShooterConstants.ROLLER_SPEED_PERCENT),
						this));
	}

	public void setAutoTargetsFromDistanceFeet(double distanceFeet) {
		autoDistanceFeet = distanceFeet;
		autoThreeInchPercent = ShooterAutoMap.getThreeInchPercent(distanceFeet);
		setTargets(
				ShooterAutoMap.getTwoInchRpm(distanceFeet),
				ShooterAutoMap.getThreeInchRpm(distanceFeet),
				ShooterConstants.NEO_550_SPEED_PERCENT,
				ShooterConstants.ROLLER_SPEED_PERCENT);
	}

	public Command runAutoShoot(DoubleSupplier distanceFeetSupplier) {
		return Commands.sequence(
				Commands.runOnce(
						() -> setTargets(
								0.0,
								0.0,
								0.0,
								ShooterConstants.ROLLER_SPEED_PERCENT),
						this),
				Commands.waitSeconds(ShooterConstants.SHOOT_DELAY_SECONDS),
				Commands.run(
						() -> setAutoTargetsFromDistanceFeet(distanceFeetSupplier.getAsDouble()),
						this));
	}

	public Command runCoralIntake() {
		return runShoot();
	}

	public Command stopCoralIntake() {
		return Commands.runOnce(this::stop, this);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		if (enabled) {
			io.setTargets(target2InchRpm, target3InchRpm, targetNeoPercent, targetRollerPercent);
		}

		Logger.recordOutput("Shooter/Enabled", enabled);
		Logger.recordOutput("Shooter/Target2InchRPM", target2InchRpm);
		Logger.recordOutput("Shooter/Target3InchRPM", target3InchRpm);
		Logger.recordOutput("Shooter/TargetNeoPercent", targetNeoPercent);
		Logger.recordOutput("Shooter/TargetRollerPercent", targetRollerPercent);
		Logger.recordOutput("Shooter/AutoDistanceFeet", autoDistanceFeet);
		Logger.recordOutput("Shooter/Auto3InchPercent", autoThreeInchPercent);
	}
}
