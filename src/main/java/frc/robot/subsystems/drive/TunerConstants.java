// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;


public class TunerConstants {
	//gains still need to be tuned
	private static final Slot0Configs steerGains = new Slot0Configs()
		.withKP(100).withKI(0).withKD(0.5)
		.withKS(0.1).withKV(2.49).withKA(0)
		.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
	
	private static final Slot0Configs driveGains = new Slot0Configs()
		.withKP(0.1).withKI(0).withKD(0)
		.withKS(0).withKV(0.124);

	
	private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
	
	private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

	
	private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
	
	private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

	
	private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

	
	private static final Current kSlipCurrent = Amps.of(120);

	
	private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
	private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
		.withCurrentLimits(
			new CurrentLimitsConfigs()
				
				.withStatorCurrentLimit(Amps.of(60))
				.withStatorCurrentLimitEnable(true)
		);
	private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
	
	private static final Pigeon2Configuration pigeonConfigs = null;

	
	public static final CANBus kCANBus = new CANBus("Default Name");

	//needs to be tuned
	public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.12);

	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
	// needs to be tuned still 
	private static final double kCoupleRatio = 3.857142857142857;

	private static final double kDriveGearRatio = 6.026785714285714;
	private static final double kSteerGearRatio = 26.09090909090909;
	private static final Distance kWheelRadius = Inches.of(2);

	private static final boolean kInvertLeftSide = false;
	private static final boolean kInvertRightSide = true;

	private static final int kPigeonId = 0;

	// These are only used for simulation
	private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
	private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
	// Simulated voltage necessary to overcome friction
	private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
	private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

	public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
			.withCANBusName(kCANBus.getName())
			.withPigeon2Id(kPigeonId)
			.withPigeon2Configs(pigeonConfigs);

	private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
		new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
			.withDriveMotorGearRatio(kDriveGearRatio)
			.withSteerMotorGearRatio(kSteerGearRatio)
			.withCouplingGearRatio(kCoupleRatio)
			.withWheelRadius(kWheelRadius)
			.withSteerMotorGains(steerGains)
			.withDriveMotorGains(driveGains)
			.withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
			.withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
			.withSlipCurrent(kSlipCurrent)
			.withSpeedAt12Volts(kSpeedAt12Volts)
			.withDriveMotorType(kDriveMotorType)
			.withSteerMotorType(kSteerMotorType)
			.withFeedbackSource(kSteerFeedbackType)
			.withDriveMotorInitialConfigs(driveInitialConfigs)
			.withSteerMotorInitialConfigs(steerInitialConfigs)
			.withEncoderInitialConfigs(encoderInitialConfigs)
			.withSteerInertia(kSteerInertia)
			.withDriveInertia(kDriveInertia)
			.withSteerFrictionVoltage(kSteerFrictionVoltage)
			.withDriveFrictionVoltage(kDriveFrictionVoltage);


	// Front Left
	private static final int kFrontLeftDriveMotorId = 11;
	private static final int kFrontLeftSteerMotorId = 13;
	private static final int kFrontLeftEncoderId = 12;
	private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.049560546875);
	private static final boolean kFrontLeftSteerMotorInverted = false;
	private static final boolean kFrontLeftEncoderInverted = false;

	private static final Distance kFrontLeftXPos = Inches.of(11.375);
	private static final Distance kFrontLeftYPos = Inches.of(11.375);

	// Front Right
	private static final int kFrontRightDriveMotorId = 14;
	private static final int kFrontRightSteerMotorId = 16;
	private static final int kFrontRightEncoderId = 15;
	private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.414306640625);
	private static final boolean kFrontRightSteerMotorInverted = false;
	private static final boolean kFrontRightEncoderInverted = false;

	private static final Distance kFrontRightXPos = Inches.of(11.375);
	private static final Distance kFrontRightYPos = Inches.of(-11.375);

	// Back Left
	private static final int kBackLeftDriveMotorId = 21;
	private static final int kBackLeftSteerMotorId = 23;
	private static final int kBackLeftEncoderId = 22;
	private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.393798828125);
	private static final boolean kBackLeftSteerMotorInverted = false;
	private static final boolean kBackLeftEncoderInverted = false;

	private static final Distance kBackLeftXPos = Inches.of(-11.375);
	private static final Distance kBackLeftYPos = Inches.of(11.375);

	// Back Right
	private static final int kBackRightDriveMotorId = 24;
	private static final int kBackRightSteerMotorId = 26;
	private static final int kBackRightEncoderId = 25;
	private static final Angle kBackRightEncoderOffset = Rotations.of(0.0537109375);
	private static final boolean kBackRightSteerMotorInverted = false;
	private static final boolean kBackRightEncoderInverted = false;

	private static final Distance kBackRightXPos = Inches.of(-11.375);
	private static final Distance kBackRightYPos = Inches.of(-11.375);

	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
			.createModuleConstants(
					kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
					kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted,
					kFrontLeftEncoderInverted);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
			.createModuleConstants(
					kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
					kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted,
					kFrontRightEncoderInverted);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
			.createModuleConstants(
					kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
					kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted,
					kBackLeftEncoderInverted);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
			.createModuleConstants(
					kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
					kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted,
					kBackRightEncoderInverted);

	
	public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants,
				SwerveModuleConstants<?, ?, ?>... modules) {
			super(
					TalonFX::new, TalonFX::new, CANcoder::new,
					drivetrainConstants, modules);
		}

		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants,
				double odometryUpdateFrequency,
				SwerveModuleConstants<?, ?, ?>... modules) {
			super(
					TalonFX::new, TalonFX::new, CANcoder::new,
					drivetrainConstants, odometryUpdateFrequency, modules);
		}

		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants,
				double odometryUpdateFrequency,
				Matrix<N3, N1> odometryStandardDeviation,
				Matrix<N3, N1> visionStandardDeviation,
				SwerveModuleConstants<?, ?, ?>... modules) {
			super(
					TalonFX::new, TalonFX::new, CANcoder::new,
					drivetrainConstants, odometryUpdateFrequency,
					odometryStandardDeviation, visionStandardDeviation, modules);
		}
	}
}
