package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
	private final IntakeArmIO io;
	private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

	private double targetAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;
	private boolean enabled = true;

	public IntakeArm(IntakeArmIO io) {
		this.io = io;
	}

	public void setTargetAngleDeg(double targetAngleDeg) {
		this.targetAngleDeg = targetAngleDeg;
		enabled = true;
		io.setTargetAngleDeg(targetAngleDeg);
	}

	public void moveUp() {
		setTargetAngleDeg(IntakeArmConstants.ARM_UP_ANGLE_DEG);
	}

	public void moveDown() {
		setTargetAngleDeg(IntakeArmConstants.ARM_DOWN_ANGLE_DEG);
	}

	public void stop() {
		enabled = false;
		io.stop();
	}

	public Command moveUpCommand() {
		return Commands.runOnce(this::moveUp, this);
	}

	public Command moveDownCommand() {
		return Commands.runOnce(this::moveDown, this);
	}

	@AutoLogOutput(key = "IntakeArm/AtTarget")
	public boolean atTarget() {
		double errorDeg = MathUtil.inputModulus(inputs.targetAngleDeg - inputs.absoluteAngleDeg, -180.0, 180.0);
		return Math.abs(errorDeg) <= IntakeArmConstants.ARM_ANGLE_TOLERANCE_DEG;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("IntakeArm", inputs);

		if (enabled) {
			io.setTargetAngleDeg(targetAngleDeg);
		}

		Logger.recordOutput("IntakeArm/Enabled", enabled);
		Logger.recordOutput("IntakeArm/TargetAngleDeg", targetAngleDeg);
	}
}
