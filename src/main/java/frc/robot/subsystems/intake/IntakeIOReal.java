package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.TunerConstants;

public class IntakeIOReal implements IntakeIO {
	private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);

	private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
	private final NeutralOut neutral = new NeutralOut();

	private final StatusSignal<Voltage> motorVoltage = intakeMotor.getMotorVoltage();
	private final StatusSignal<Current> statorCurrent = intakeMotor.getStatorCurrent();

	public IntakeIOReal() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT_A;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_A;
		tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(config, 0.25));
	}

	@Override
	public void setPercent(double percent) {
		double clampedPercent = MathUtil.clamp(percent, -100.0, 100.0);
		double output = (clampedPercent / 100.0) * IntakeConstants.INTAKE_DIRECTION;
		intakeMotor.setControl(dutyCycleOut.withOutput(output));
	}

	@Override
	public void stop() {
		intakeMotor.setControl(neutral);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.connected = BaseStatusSignal.refreshAll(motorVoltage, statorCurrent).isOK();
		inputs.appliedVolts = motorVoltage.getValueAsDouble();
		inputs.currentAmps = statorCurrent.getValueAsDouble();
		inputs.appliedPercent = (inputs.appliedVolts / 12.0) * 100.0;
	}
}
