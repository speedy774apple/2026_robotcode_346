package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {
	private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, Constants.SUBSYSTEMS_CAN_BUS);
	private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);
	private final NeutralOut neutralRequest = new NeutralOut();

	private final StatusSignal<Angle> motorPosition = climbMotor.getPosition();
	private final StatusSignal<AngularVelocity> motorVelocity = climbMotor.getVelocity();
	private final StatusSignal<Voltage> motorVoltage = climbMotor.getMotorVoltage();
	private final StatusSignal<Current> motorCurrent = climbMotor.getStatorCurrent();

	private double targetMotorRotations = 0.0;
	private boolean brakeModeConfigured = false;

	public ClimbIOReal() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.PeakForwardDutyCycle = ClimbConstants.MAX_DUTY_CYCLE;
		config.MotorOutput.PeakReverseDutyCycle = -ClimbConstants.MAX_DUTY_CYCLE;
		config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ClimbConstants.CLOSED_LOOP_RAMP_SECONDS;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT_AMPS;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT_AMPS;
		config.Slot0.kP = ClimbConstants.SLOT0_KP;
		config.Slot0.kD = ClimbConstants.SLOT0_KD;

		tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config, 0.25));
		brakeModeConfigured = ensureBrakeModeConfigured();
		climbMotor.setControl(neutralRequest);
	}

	private boolean ensureBrakeModeConfigured() {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		tryUntilOk(5, () -> climbMotor.getConfigurator().refresh(motorOutputConfigs, 0.25));

		if (motorOutputConfigs.NeutralMode != NeutralModeValue.Brake) {
			motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
			tryUntilOk(5, () -> climbMotor.getConfigurator().apply(motorOutputConfigs, 0.25));
			DriverStation.reportWarning("Climb neutral mode was not Brake. Re-applied Brake mode.", false);
			tryUntilOk(5, () -> climbMotor.getConfigurator().refresh(motorOutputConfigs, 0.25));
		}

		return motorOutputConfigs.NeutralMode == NeutralModeValue.Brake;
	}

	@Override
	public void setTargetPositionRotations(double targetPositionRotations) {
		targetMotorRotations = targetPositionRotations;
		climbMotor.setControl(positionRequest.withPosition(targetMotorRotations));
	}

	@Override
	public void stop() {
		climbMotor.setControl(neutralRequest);
	}

	@Override
	public void updateInputs(ClimbIOInputs inputs) {
		inputs.connected = BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorVoltage, motorCurrent).isOK();
		inputs.motorPositionRotations = motorPosition.getValueAsDouble();
		inputs.motorVelocityRps = motorVelocity.getValueAsDouble();
		inputs.targetPositionRotations = targetMotorRotations;
		inputs.appliedVolts = motorVoltage.getValueAsDouble();
		inputs.currentAmps = motorCurrent.getValueAsDouble();
		inputs.brakeModeConfigured = brakeModeConfigured;
	}
}
