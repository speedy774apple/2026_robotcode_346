package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
	private final TalonFX talon2Inch = new TalonFX(ShooterConstants.TALON_2_INCH_ID, Constants.SUBSYSTEMS_CAN_BUS);
	private final TalonFX talon3Inch1 = new TalonFX(ShooterConstants.TALON_3_INCH_1_ID, Constants.SUBSYSTEMS_CAN_BUS);
	private final TalonFX talon3Inch2 = new TalonFX(ShooterConstants.TALON_3_INCH_2_ID, Constants.SUBSYSTEMS_CAN_BUS);
	private final TalonFX roller = new TalonFX(ShooterConstants.FEEDER_ROLLER_ID, Constants.DRIVETRAIN_CAN_BUS);
	private final TalonFXS neo550 = new TalonFXS(ShooterConstants.NEO_550_ID, Constants.SUBSYSTEMS_CAN_BUS);

	private final VelocityVoltage velReq2Inch = new VelocityVoltage(0.0).withSlot(0);
	private final VelocityVoltage velReq3Inch1 = new VelocityVoltage(0.0).withSlot(0);
	private final VelocityVoltage velReq3Inch2 = new VelocityVoltage(0.0).withSlot(0);
	private final DutyCycleOut rollerDutyCycleOut = new DutyCycleOut(0.0);
	private final NeutralOut neutral = new NeutralOut();

	private final StatusSignal<AngularVelocity> vel2Inch = talon2Inch.getVelocity();
	private final StatusSignal<AngularVelocity> vel3Inch1 = talon3Inch1.getVelocity();
	private final StatusSignal<AngularVelocity> vel3Inch2 = talon3Inch2.getVelocity();

	private final StatusSignal<Voltage> volts2Inch = talon2Inch.getMotorVoltage();
	private final StatusSignal<Voltage> volts3Inch1 = talon3Inch1.getMotorVoltage();
	private final StatusSignal<Voltage> volts3Inch2 = talon3Inch2.getMotorVoltage();
	private final StatusSignal<Voltage> voltsRoller = roller.getMotorVoltage();

	private final StatusSignal<Current> amps2Inch = talon2Inch.getStatorCurrent();
	private final StatusSignal<Current> amps3Inch1 = talon3Inch1.getStatorCurrent();
	private final StatusSignal<Current> amps3Inch2 = talon3Inch2.getStatorCurrent();
	private final StatusSignal<Current> ampsRoller = roller.getStatorCurrent();

	public ShooterIOReal() {
		TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
		shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT_A;
		shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT_A;
		shooterConfig.Slot0.kP = ShooterConstants.SHOOTER_KP;
		shooterConfig.Slot0.kI = ShooterConstants.SHOOTER_KI;
		shooterConfig.Slot0.kD = ShooterConstants.SHOOTER_KD;
		// Feedforward applied in code to make the ramp more immediate.
		shooterConfig.Slot0.kV = 0.0;
		shooterConfig.Slot0.kS = 0.0;
		shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
		shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;

		TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
		rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT_A;
		rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		rollerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT_A;
		rollerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
		rollerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ShooterConstants.FEEDER_ROLLER_OPEN_LOOP_RAMP_SECONDS;

		tryUntilOk(5, () -> talon2Inch.getConfigurator().apply(shooterConfig, 0.25));
		tryUntilOk(5, () -> talon3Inch1.getConfigurator().apply(shooterConfig, 0.25));
		tryUntilOk(5, () -> talon3Inch2.getConfigurator().apply(shooterConfig, 0.25));
		tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

		TalonFXSConfiguration neoConfig = new TalonFXSConfiguration();
		neoConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
		neoConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		neoConfig.CurrentLimits.StatorCurrentLimit = 25.0;
		neoConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		neoConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
		neoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		tryUntilOk(5, () -> neo550.getConfigurator().apply(neoConfig, 0.25));
	}

	private static double rpmToRps(double rpm) {
		return rpm / 60.0;
	}

	private static double calcFeedforwardVolts(double velocityRps) {
		if (Math.abs(velocityRps) < 1e-6) {
			return 0.0;
		}
		return ShooterConstants.SHOOTER_KS * Math.signum(velocityRps)
				+ ShooterConstants.SHOOTER_KV * velocityRps;
	}

	@Override
	public void setTargets(double twoInchRpm, double threeInchRpm, double neoPercent, double rollerPercent) {
		double target2InchRps = rpmToRps(twoInchRpm);
		double target3InchRps = rpmToRps(threeInchRpm);

		double target2InchRpsSigned = target2InchRps * ShooterConstants.TALON_2_INCH_DIR;
		double target3Inch1RpsSigned = target3InchRps * ShooterConstants.TALON_3_INCH_1_DIR;
		double target3Inch2RpsSigned = target3InchRps * ShooterConstants.TALON_3_INCH_2_DIR;

		double ff2InchVolts = calcFeedforwardVolts(target2InchRpsSigned);
		double ff3Inch1Volts = calcFeedforwardVolts(target3Inch1RpsSigned);
		double ff3Inch2Volts = calcFeedforwardVolts(target3Inch2RpsSigned);

		talon2Inch.setControl(velReq2Inch.withVelocity(target2InchRpsSigned).withFeedForward(ff2InchVolts));
		talon3Inch1.setControl(velReq3Inch1.withVelocity(target3Inch1RpsSigned).withFeedForward(ff3Inch1Volts));
		talon3Inch2.setControl(velReq3Inch2.withVelocity(target3Inch2RpsSigned).withFeedForward(ff3Inch2Volts));

		neo550.set(neoPercent / 100.0);
		double rollerOutput = (MathUtil.clamp(rollerPercent, -100.0, 100.0) / 100.0) * ShooterConstants.FEEDER_ROLLER_DIR;
		roller.setControl(rollerDutyCycleOut.withOutput(rollerOutput));
	}

	@Override
	public void stop() {
		talon2Inch.setControl(neutral);
		talon3Inch1.setControl(neutral);
		talon3Inch2.setControl(neutral);
		roller.setControl(neutral);
		neo550.set(0.0);
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.connected2Inch = BaseStatusSignal.refreshAll(vel2Inch, volts2Inch, amps2Inch).isOK();
		inputs.connected3Inch1 = BaseStatusSignal.refreshAll(vel3Inch1, volts3Inch1, amps3Inch1).isOK();
		inputs.connected3Inch2 = BaseStatusSignal.refreshAll(vel3Inch2, volts3Inch2, amps3Inch2).isOK();
		inputs.connectedRoller = BaseStatusSignal.refreshAll(voltsRoller, ampsRoller).isOK();
		inputs.connectedNeo550 = true;

		inputs.velocity2InchRps = vel2Inch.getValueAsDouble();
		inputs.velocity3Inch1Rps = vel3Inch1.getValueAsDouble();
		inputs.velocity3Inch2Rps = vel3Inch2.getValueAsDouble();

		inputs.appliedVolts2Inch = volts2Inch.getValueAsDouble();
		inputs.appliedVolts3Inch1 = volts3Inch1.getValueAsDouble();
		inputs.appliedVolts3Inch2 = volts3Inch2.getValueAsDouble();
		inputs.appliedVoltsRoller = voltsRoller.getValueAsDouble();

		inputs.currentAmps2Inch = amps2Inch.getValueAsDouble();
		inputs.currentAmps3Inch1 = amps3Inch1.getValueAsDouble();
		inputs.currentAmps3Inch2 = amps3Inch2.getValueAsDouble();
		inputs.currentAmpsRoller = ampsRoller.getValueAsDouble();
		inputs.rollerAppliedPercent = (inputs.appliedVoltsRoller / 12.0) * 100.0;
	}
}
