package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
	private static final double ARM_MIN_ANGLE_DEG =
			Math.min(IntakeArmConstants.ARM_UP_ANGLE_DEG, IntakeArmConstants.ARM_DOWN_ANGLE_DEG);
	private static final double ARM_MAX_ANGLE_DEG =
			Math.max(IntakeArmConstants.ARM_UP_ANGLE_DEG, IntakeArmConstants.ARM_DOWN_ANGLE_DEG);

	private enum IntakeArmState {
		IDLE,
		MOVING_UP,
		MOVING_DOWN,
		HOLDING,
		MANUAL
	}

	private final IntakeArmIO io;
	private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

	private double targetAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;
	private IntakeArmState state = IntakeArmState.IDLE;
	private boolean startupTargetLatched = false;

	public IntakeArm(IntakeArmIO io) {
		this.io = io;
	}

	public void setTargetAngleDeg(double targetAngleDeg) {
		this.targetAngleDeg = MathUtil.clamp(
				targetAngleDeg,
				ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG,
				ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG);
		state = IntakeArmState.HOLDING;
	}

	public void moveUp() {
		targetAngleDeg = MathUtil.clamp(
				IntakeArmConstants.ARM_UP_ANGLE_DEG,
				ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG,
				ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG);
		state = IntakeArmState.MOVING_UP;
	}

	public void moveDown() {
		targetAngleDeg = MathUtil.clamp(
				IntakeArmConstants.ARM_DOWN_ANGLE_DEG,
				ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG,
				ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG);
		state = IntakeArmState.MOVING_DOWN;
	}

	public void jogUpStep() {
		double signedStep = IntakeArmConstants.ARM_JOG_STEP_DEG
				* Math.signum(IntakeArmConstants.ARM_DOWN_ANGLE_DEG - IntakeArmConstants.ARM_UP_ANGLE_DEG);
		targetAngleDeg = MathUtil.clamp(
				targetAngleDeg - signedStep,
				ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG,
				ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG);
		state = IntakeArmState.MOVING_UP;
	}

	public void jogDownStep() {
		double signedStep = IntakeArmConstants.ARM_JOG_STEP_DEG
				* Math.signum(IntakeArmConstants.ARM_DOWN_ANGLE_DEG - IntakeArmConstants.ARM_UP_ANGLE_DEG);
		targetAngleDeg = MathUtil.clamp(
				targetAngleDeg + signedStep,
				ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG,
				ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG);
		state = IntakeArmState.MOVING_DOWN;
	}

	public void stop() {
		state = IntakeArmState.IDLE;
		io.stop();
	}

	private void setManualState(double outputPercent) {
		state = IntakeArmState.MANUAL;
		io.setManualOutputPercent(outputPercent);
	}

	public void manualUp() {
		if (inputs.connectedEncoder) {
			if (inputs.absoluteAngleDeg >= (ARM_MAX_ANGLE_DEG - IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG)) {
				io.stop();
				state = IntakeArmState.IDLE;
				return;
			}
		}
		setManualState(IntakeArmConstants.ARM_MANUAL_OUTPUT_PERCENT);
	}

	public void manualDown() {
		if (inputs.connectedEncoder) {
			if (inputs.absoluteAngleDeg <= (ARM_MIN_ANGLE_DEG + IntakeArmConstants.ARM_SOFT_LIMIT_BUFFER_DEG)) {
				io.stop();
				state = IntakeArmState.IDLE;
				return;
			}
		}
		setManualState(-IntakeArmConstants.ARM_MANUAL_OUTPUT_PERCENT);
	}

	public Command moveUpCommand() {
		return Commands.runOnce(this::moveUp, this);
	}

	public Command moveDownCommand() {
		return Commands.runOnce(this::moveDown, this);
	}

	public Command jogUpCommand() {
		return IntakeArmConstants.ARM_TUNING_MODE
				? Commands.run(this::manualUp, this)
				: Commands.run(this::jogUpStep, this);
	}

	public Command jogDownCommand() {
		return IntakeArmConstants.ARM_TUNING_MODE
				? Commands.run(this::manualDown, this)
				: Commands.run(this::jogDownStep, this);
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

		// Latch the current measured angle on startup so the arm holds where it is
		// instead of jumping to a preset immediately. Keep IDLE so it does not
		// actively drive unless commanded by operator input.
		if (!startupTargetLatched && inputs.connectedEncoder) {
			targetAngleDeg = inputs.absoluteAngleDeg;
			state = IntakeArmState.IDLE;
			startupTargetLatched = true;
		}

		switch (state) {
			case IDLE -> io.stop();
			case MANUAL -> { }
			case MOVING_UP, MOVING_DOWN -> {
				io.setTargetAngleDeg(targetAngleDeg);
				if (atTarget()) {
					state = IntakeArmState.HOLDING;
				}
			}
			case HOLDING -> io.setTargetAngleDeg(targetAngleDeg);
		}

		Logger.recordOutput("IntakeArm/State", state.toString());
		Logger.recordOutput("IntakeArm/TargetAngleDeg", targetAngleDeg);
		Logger.recordOutput("Tuning/IntakeArm/AbsRotRaw", inputs.encoderAbsoluteRot);
		Logger.recordOutput("Tuning/IntakeArm/AbsDegRaw", inputs.encoderAbsoluteDeg);
		Logger.recordOutput("Tuning/IntakeArm/AbsDegAdjusted", inputs.absoluteAngleDeg);
		Logger.recordOutput("Tuning/IntakeArm/ArmMinAngleDeg", ARM_MIN_ANGLE_DEG);
		Logger.recordOutput("Tuning/IntakeArm/ArmMaxAngleDeg", ARM_MAX_ANGLE_DEG);
	}
}
