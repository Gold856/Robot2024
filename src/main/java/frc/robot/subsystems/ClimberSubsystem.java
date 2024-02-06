// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {

	private static ClimberSubsystem s_subsystem;

	private final CANSparkMax m_leftMotor = new CANSparkMax(ClimbConstants.kLeftPort, MotorType.kBrushless);
	private final CANSparkMax m_rightMotor = new CANSparkMax(ClimbConstants.kRightPort, MotorType.kBrushless);

	private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
	private final SparkPIDController m_leftPidController = m_leftMotor.getPIDController();

	private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
	private final SparkPIDController m_rightPidController = m_rightMotor.getPIDController();

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
		s_subsystem = this;

		m_leftMotor.restoreFactoryDefaults();
		m_leftMotor.setInverted(ClimbConstants.kLeftInvert);
		m_leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_leftMotor.enableVoltageCompensation(12);
		m_leftMotor.setSmartCurrentLimit(ClimbConstants.kSmartCurrentLimit);

		m_rightMotor.restoreFactoryDefaults();
		m_rightMotor.setInverted(ClimbConstants.kRightInvert);
		m_rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_rightMotor.enableVoltageCompensation(12);
		m_rightMotor.setSmartCurrentLimit(ClimbConstants.kSmartCurrentLimit);

		m_leftPidController.setP(ClimbConstants.kP);
		m_leftPidController.setI(ClimbConstants.kI);
		m_leftPidController.setD(ClimbConstants.kD);
		m_leftPidController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

		m_rightPidController.setP(ClimbConstants.kP);
		m_rightPidController.setI(ClimbConstants.kI);
		m_rightPidController.setD(ClimbConstants.kD);
		m_rightPidController.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
