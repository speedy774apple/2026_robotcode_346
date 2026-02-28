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
import frc.robot.subsystems.drive.TunerConstants;

public class ShooterIOReal implements ShooterIO {
	private final TalonFX talon2Inch1 = new TalonFX(ShooterConstants.TALON_2_INCH_1_ID, TunerConstants.DrivetrainConstants.CANBusName);
	private final TalonFX talon2Inch2 = new TalonFX(ShooterConstants.TALON_2_INCH_2_ID, TunerConstants.DrivetrainConstants.CANBusName);
	private final TalonFX talon3Inch1 = new TalonFX(ShooterConstants.TALON_3_INCH_1_ID, TunerConstants.DrivetrainConstants.CANBusName);
	private final TalonFX talon3Inch2 = new TalonFX(ShooterConstants.TALON_3_INCH_2_ID, TunerConstants.DrivetrainConstants.CANBusName);
	private final TalonFX roller = new TalonFX(ShooterConstants.ROLLER_ID, TunerConstants.DrivetrainConstants.CANBusName);
	private final TalonFXS neo550 = new TalonFXS(ShooterConstants.NEO_550_ID, TunerConstants.DrivetrainConstants.CANBusName);

	private final VelocityVoltage velReq2Inch1 = new VelocityVoltage(0.0).withSlot(0);
	private final VelocityVoltage velReq2Inch2 = new VelocityVoltage(0.0).withSlot(0);
	private final VelocityVoltage velReq3Inch1 = new VelocityVoltage(0.0).withSlot(0);
	private final VelocityVoltage velReq3Inch2 = new VelocityVoltage(0.0).withSlot(0);
	private final DutyCycleOut rollerDutyCycleOut = new DutyCycleOut(0.0);
	private final NeutralOut neutral = new NeutralOut();

	private final StatusSignal<AngularVelocity> vel2Inch1 = talon2Inch1.getVelocity();
	private final StatusSignal<AngularVelocity> vel2Inch2 = talon2Inch2.getVelocity();
	private final StatusSignal<AngularVelocity> vel3Inch1 = talon3Inch1.getVelocity();
	private final StatusSignal<AngularVelocity> vel3Inch2 = talon3Inch2.getVelocity();

	private final StatusSignal<Voltage> volts2Inch1 = talon2Inch1.getMotorVoltage();
	private final StatusSignal<Voltage> volts2Inch2 = talon2Inch2.getMotorVoltage();
	private final StatusSignal<Voltage> volts3Inch1 = talon3Inch1.getMotorVoltage();
	private final StatusSignal<Voltage> volts3Inch2 = talon3Inch2.getMotorVoltage();
	private final StatusSignal<Voltage> voltsRoller = roller.getMotorVoltage();

	private final StatusSignal<Current> amps2Inch1 = talon2Inch1.getStatorCurrent();
	private final StatusSignal<Current> amps2Inch2 = talon2Inch2.getStatorCurrent();
	private final StatusSignal<Current> amps3Inch1 = talon3Inch1.getStatorCurrent();
	private final StatusSignal<Current> amps3Inch2 = talon3Inch2.getStatorCurrent();
	private final StatusSignal<Current> ampsRoller = roller.getStatorCurrent();

	public ShooterIOReal() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT_A;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT_A;
		config.Slot0.kP = ShooterConstants.SHOOTER_KP;
		config.Slot0.kI = ShooterConstants.SHOOTER_KI;
		config.Slot0.kD = ShooterConstants.SHOOTER_KD;
		config.Slot0.kV = ShooterConstants.SHOOTER_KV;
		config.Slot0.kS = ShooterConstants.SHOOTER_KS;
		config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
		config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;

		tryUntilOk(5, () -> talon2Inch1.getConfigurator().apply(config, 0.25));
		tryUntilOk(5, () -> talon2Inch2.getConfigurator().apply(config, 0.25));
		tryUntilOk(5, () -> talon3Inch1.getConfigurator().apply(config, 0.25));
		tryUntilOk(5, () -> talon3Inch2.getConfigurator().apply(config, 0.25));
		tryUntilOk(5, () -> roller.getConfigurator().apply(config, 0.25));

		TalonFXSConfiguration neoConfig = new TalonFXSConfiguration();
		neoConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
		neoConfig.CurrentLimits.StatorCurrentLimit = 25.0;
		neoConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		neoConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
		neoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		tryUntilOk(5, () -> neo550.getConfigurator().apply(neoConfig, 0.25));
	}

	private static double rpmToRps(double rpm) {
		return rpm / 60.0;
	}

	@Override
	public void setTargets(double twoInchRpm, double threeInchRpm, double neoPercent, double rollerPercent) {
		double target2InchRps = rpmToRps(twoInchRpm);
		double target3InchRps = rpmToRps(threeInchRpm);

		talon2Inch1.setControl(velReq2Inch1.withVelocity(target2InchRps * ShooterConstants.TALON_2_INCH_1_DIR));
		talon2Inch2.setControl(velReq2Inch2.withVelocity(target2InchRps * ShooterConstants.TALON_2_INCH_2_DIR));
		talon3Inch1.setControl(velReq3Inch1.withVelocity(target3InchRps * ShooterConstants.TALON_3_INCH_1_DIR));
		talon3Inch2.setControl(velReq3Inch2.withVelocity(target3InchRps * ShooterConstants.TALON_3_INCH_2_DIR));

		neo550.set(neoPercent / 100.0);
		double rollerOutput = (MathUtil.clamp(rollerPercent, -100.0, 100.0) / 100.0) * ShooterConstants.ROLLER_DIR;
		roller.setControl(rollerDutyCycleOut.withOutput(rollerOutput));
	}

	@Override
	public void stop() {
		talon2Inch1.setControl(neutral);
		talon2Inch2.setControl(neutral);
		talon3Inch1.setControl(neutral);
		talon3Inch2.setControl(neutral);
		roller.setControl(neutral);
		neo550.set(0.0);
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.connected2Inch1 = BaseStatusSignal.refreshAll(vel2Inch1, volts2Inch1, amps2Inch1).isOK();
		inputs.connected2Inch2 = BaseStatusSignal.refreshAll(vel2Inch2, volts2Inch2, amps2Inch2).isOK();
		inputs.connected3Inch1 = BaseStatusSignal.refreshAll(vel3Inch1, volts3Inch1, amps3Inch1).isOK();
		inputs.connected3Inch2 = BaseStatusSignal.refreshAll(vel3Inch2, volts3Inch2, amps3Inch2).isOK();
		inputs.connectedRoller = BaseStatusSignal.refreshAll(voltsRoller, ampsRoller).isOK();
		inputs.connectedNeo550 = true;

		inputs.velocity2Inch1Rps = vel2Inch1.getValueAsDouble();
		inputs.velocity2Inch2Rps = vel2Inch2.getValueAsDouble();
		inputs.velocity3Inch1Rps = vel3Inch1.getValueAsDouble();
		inputs.velocity3Inch2Rps = vel3Inch2.getValueAsDouble();

		inputs.appliedVolts2Inch1 = volts2Inch1.getValueAsDouble();
		inputs.appliedVolts2Inch2 = volts2Inch2.getValueAsDouble();
		inputs.appliedVolts3Inch1 = volts3Inch1.getValueAsDouble();
		inputs.appliedVolts3Inch2 = volts3Inch2.getValueAsDouble();
		inputs.appliedVoltsRoller = voltsRoller.getValueAsDouble();

		inputs.currentAmps2Inch1 = amps2Inch1.getValueAsDouble();
		inputs.currentAmps2Inch2 = amps2Inch2.getValueAsDouble();
		inputs.currentAmps3Inch1 = amps3Inch1.getValueAsDouble();
		inputs.currentAmps3Inch2 = amps3Inch2.getValueAsDouble();
		inputs.currentAmpsRoller = ampsRoller.getValueAsDouble();
		inputs.rollerAppliedPercent = (inputs.appliedVoltsRoller / 12.0) * 100.0;
	}
}
