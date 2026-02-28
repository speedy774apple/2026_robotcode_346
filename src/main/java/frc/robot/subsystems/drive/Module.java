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

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;
	private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

	private final Alert driveDisconnectedAlert;
	private final Alert turnDisconnectedAlert;
	private final Alert turnEncoderDisconnectedAlert;
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public Module(
			ModuleIO io,
			int index,
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
		this.io = io;
		this.index = index;
		this.constants = constants;
		driveDisconnectedAlert = new Alert(
				"Disconnected drive motor on module " + Integer.toString(index) + ".",
				AlertType.kError);
		turnDisconnectedAlert = new Alert(
				"Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
		turnEncoderDisconnectedAlert = new Alert(
				"Disconnected turn encoder on module " + Integer.toString(index) + ".",
				AlertType.kError);
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		// Calculate positions for odometry
		int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		// Update alerts
		driveDisconnectedAlert.set(!inputs.driveConnected);
		turnDisconnectedAlert.set(!inputs.turnConnected);
		turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
	}

	/**
	 * Runs the module with the specified setpoint state. Mutates the state to
	 * optimize it.
	 * 
	 * IMPORTANT: This method correctly:
	 * 1. Optimizes the wheel angle to minimize rotation (using current angle)
	 * 2. Applies cosine scaling to prevent wheel slip during turns
	 * 3. Converts speed to voltage for open-loop drive control
	 * 4. Sets both drive voltage and turn angle correctly
	 */
	public void runSetpoint(SwerveModuleState state) {
		// Get current wheel angle BEFORE optimization
		Rotation2d currentAngle = getAngle();
		
		// Optimize velocity setpoint - minimizes wheel rotation by choosing
		// the closest equivalent angle (current angle ± 180°)
		state.optimize(currentAngle);
		
		// Apply cosine scaling to prevent wheel slip when turning
		// This scales the drive speed based on how much the wheel needs to turn
		// If the wheel is already at the target angle, full speed is applied
		// If the wheel needs to turn 90°, speed is scaled to 0
		state.cosineScale(currentAngle);

		// Convert speed (m/s) to voltage for open-loop drive control
		// This matches the CTRE generated code approach
		double maxSpeedMetersPerSec = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		
		// Normalize speed to -1.0 to 1.0, then multiply by 12V
		// Clamp to prevent over-voltage
		double normalizedSpeed = state.speedMetersPerSecond / maxSpeedMetersPerSec;
		double driveVoltageOutput = Math.max(-12.0, Math.min(12.0, normalizedSpeed * 12.0));
		
		// Apply setpoints to hardware
		io.setDriveOpenLoop(driveVoltageOutput);
		io.setTurnPosition(state.angle);
	}

	/**
	 * Runs the module with the specified output while controlling to zero degrees.
	 */
	public void runCharacterization(double output) {
		io.setDriveOpenLoop(output);
		io.setTurnPosition(new Rotation2d());
	}

	/** Disables all outputs to motors. */
	public void stop() {
		io.setDriveOpenLoop(0.0);
		io.setTurnOpenLoop(0.0);
	}

	/** Returns the current turn angle of the module. */
	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	/** Returns the current drive position of the module in meters. */
	public double getPositionMeters() {
		return inputs.drivePositionRad * constants.WheelRadius;
	}

	/** Returns the current drive velocity of the module in meters per second. */
	public double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadPerSec * constants.WheelRadius;
	}

	/** Returns the module position (turn angle and drive position). */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Returns the module positions received this cycle. */
	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	/** Returns the timestamps of the samples received this cycle. */
	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

	/** Returns the module position in radians. */
	public double getWheelRadiusCharacterizationPosition() {
		return inputs.drivePositionRad;
	}

	/** Returns the module velocity in rotations/sec (Phoenix native units). */
	public double getFFCharacterizationVelocity() {
		return edu.wpi.first.math.util.Units.radiansToRotations(inputs.driveVelocityRadPerSec);
	}
}
