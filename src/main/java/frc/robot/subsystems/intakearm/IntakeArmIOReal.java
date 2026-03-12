package frc.robot.subsystems.intakearm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeArmIOReal implements IntakeArmIO {
	private final TalonFX armMotor = new TalonFX(IntakeArmConstants.ARM_MOTOR_ID, Constants.SUBSYSTEMS_CAN_BUS);
	private final CANcoder armEncoder = new CANcoder(IntakeArmConstants.ARM_CANCODER_ID, Constants.DRIVETRAIN_CAN_BUS);

	private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
	private final NeutralOut neutralOut = new NeutralOut();

	private final StatusSignal<Angle> encoderAbsoluteRot = armEncoder.getAbsolutePosition();
	private final StatusSignal<Voltage> motorVolts = armMotor.getMotorVoltage();
	private final StatusSignal<Current> motorCurrent = armMotor.getStatorCurrent();

	private double targetAngleDeg = IntakeArmConstants.ARM_UP_ANGLE_DEG;

	public IntakeArmIOReal() {
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeArmConstants.ARM_MOTOR_SUPPLY_CURRENT_LIMIT_A;
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = IntakeArmConstants.ARM_MOTOR_STATOR_CURRENT_LIMIT_A;
		tryUntilOk(5, () -> armMotor.getConfigurator().apply(motorConfig, 0.25));

		CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.SensorDirection = IntakeArmConstants.ARM_CANCODER_CCW_POSITIVE
				? SensorDirectionValue.CounterClockwise_Positive
				: SensorDirectionValue.Clockwise_Positive;
		tryUntilOk(5, () -> armEncoder.getConfigurator().apply(encoderConfig, 0.25));
	}

	@Override
	public void setTargetAngleDeg(double targetAngleDeg) {
		this.targetAngleDeg = targetAngleDeg;
	}

	@Override
	public void stop() {
		armMotor.setControl(neutralOut);
	}

	@Override
	public void setManualOutputPercent(double percent) {
		double output = MathUtil.clamp(percent, -100.0, 100.0) / 100.0;
		armMotor.setControl(dutyCycleOut.withOutput(output));
	}

	@Override
	public void updateInputs(IntakeArmIOInputs inputs) {
		inputs.connectedEncoder = BaseStatusSignal.refreshAll(encoderAbsoluteRot).isOK();
		inputs.connectedMotor = BaseStatusSignal.refreshAll(motorVolts, motorCurrent).isOK();
		inputs.encoderAbsoluteRot = encoderAbsoluteRot.getValueAsDouble();
		inputs.encoderAbsoluteDeg = inputs.encoderAbsoluteRot * 360.0;

		// Safety: never drive the arm closed-loop without a valid absolute encoder reading.
		if (!inputs.connectedEncoder) {
			armMotor.setControl(neutralOut);
			inputs.absoluteAngleDeg = 0.0;
			inputs.targetAngleDeg = targetAngleDeg;
			inputs.appliedVolts = motorVolts.getValueAsDouble();
			inputs.currentAmps = motorCurrent.getValueAsDouble();
			inputs.appliedPercent = 0.0;
			return;
		}

		double relativeRotations = inputs.encoderAbsoluteRot - IntakeArmConstants.ARM_UP_REFERENCE_ROT;
		double absoluteAngleDeg = MathUtil.inputModulus(
				relativeRotations * 360.0 * IntakeArmConstants.ARM_DIRECTION,
				-180.0,
				180.0);
		double errorDeg = MathUtil.inputModulus(targetAngleDeg - absoluteAngleDeg, -180.0, 180.0);

		double output;
		if (Math.abs(errorDeg) <= IntakeArmConstants.ARM_ANGLE_TOLERANCE_DEG) {
			output = 0.0;
		} else {
			output = errorDeg * IntakeArmConstants.ARM_KP;
			output = MathUtil.clamp(output, -IntakeArmConstants.ARM_MAX_OUTPUT, IntakeArmConstants.ARM_MAX_OUTPUT);
			if (Math.abs(output) < IntakeArmConstants.ARM_MIN_MOVING_OUTPUT) {
				output = Math.copySign(IntakeArmConstants.ARM_MIN_MOVING_OUTPUT, output);
			}
		}

		armMotor.setControl(dutyCycleOut.withOutput(output));

		inputs.absoluteAngleDeg = absoluteAngleDeg;
		inputs.targetAngleDeg = targetAngleDeg;
		inputs.appliedVolts = motorVolts.getValueAsDouble();
		inputs.currentAmps = motorCurrent.getValueAsDouble();
		inputs.appliedPercent = output * 100.0;
	}
}
