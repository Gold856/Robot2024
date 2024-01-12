// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
	private PIDController m_PIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
	private CANcoder m_CANCoder;
	private CANSparkMax m_driveMotor;
	public RelativeEncoder m_driveEncoder;
	private CANSparkMax m_steerMotor;

	public SwerveModule(int CANport, int drivePort, int steerPort, double magnetOffset, boolean inverted) {
		m_CANCoder = new CANcoder(CANport);
		m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
		m_driveEncoder = m_driveMotor.getEncoder();
		m_CANCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(-magnetOffset));
		configMotorController(m_driveMotor);
		m_driveMotor.setInverted(inverted);
		configMotorController(m_steerMotor);
		m_PIDController.enableContinuousInput(0, 360);
		m_driveEncoder.setPositionConversionFactor(1 / SwerveConstants.kMotorRevsPerMeter);
	}

	/***
	 * Configures our motors with the exact same settings
	 * 
	 * @param motorController The CANSparkMax to configure
	 */
	public static void configMotorController(CANSparkMax motorController) {
		motorController.restoreFactoryDefaults();
		motorController.setIdleMode(IdleMode.kBrake);
		motorController.enableVoltageCompensation(12);
		motorController.setSmartCurrentLimit(30);
	}

	public double getDriveEncoderPosition() {
		return this.m_driveEncoder.getPosition();
	}

	/**
	 * Resets drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveEncoder.setPosition(0);
	}

	/**
	 * Gets the current drive motor speed.
	 * 
	 * @return The motor speed in percent output
	 */
	public double getDriveSpeed() {
		return m_driveMotor.getAppliedOutput();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return The module angle
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Sets the module angle in degrees.
	 * 
	 * @param angle The angle in degrees
	 */
	public void setModuleAngle(double angle) {
		m_PIDController.setSetpoint(angle);
	}

	public void setModuleState(SwerveModuleState state) {
		// Will allow the module to spin to 180 deg + target angle
		// but swap drive speed if that is quicker than normal
		state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getModuleAngle()));
		// Set drive and steer speed
		m_driveMotor.set(state.speedMetersPerSecond);
		m_steerMotor.set(m_PIDController.calculate(getModuleAngle(), state.angle.getDegrees()));
		// Print state to dashboard
		SmartDashboard.putString("Swerve module " + m_CANCoder.getDeviceID(), state.toString());
	}
}
