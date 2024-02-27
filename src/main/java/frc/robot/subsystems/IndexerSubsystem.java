// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
	private CANSparkMax m_indexerMotor = new CANSparkMax(IndexerConstants.kIndexerPort, MotorType.kBrushless);;
	private final RelativeEncoder m_encoder = m_indexerMotor.getEncoder();
	private final SparkPIDController m_controller = m_indexerMotor.getPIDController();
	private SparkLimitSwitch m_forwardLimitSwitch;

	/**
	 * Creates a new IndexerSubsystem.
	 * 
	 * 
	 */
	public IndexerSubsystem() {
		m_indexerMotor.setIdleMode(IdleMode.kBrake);
		m_indexerMotor.enableVoltageCompensation(12);
		m_indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerSmartCurrentLimit);
		m_indexerMotor.setSecondaryCurrentLimit(IndexerConstants.kIndexerPeakCurrentLimit);
		m_forwardLimitSwitch = m_indexerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		m_forwardLimitSwitch.enableLimitSwitch(false);
		m_encoder.setVelocityConversionFactor(IndexerConstants.kIndexerGearRatio);
		m_controller.setP(IndexerConstants.kP);
		m_controller.setI(IndexerConstants.kI);
		m_controller.setD(IndexerConstants.kD);
		m_controller.setIZone(IndexerConstants.kIz);
		m_controller.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);
	}

	public void setSpeed(double speed) {
		m_indexerMotor.set(speed);
	}

	/**
	 * Sets target speed for Indexer.
	 * 
	 * @param velocity Target velocity (rpm).
	 */
	public void setVelocity(double velocity) {
		m_controller.setReference(velocity, ControlType.kVelocity);
	}

	public void stop() {
		setSpeed(0.0);
	}

	public boolean getLimitSwitch() {
		return m_forwardLimitSwitch.isPressed();
	}

	public void periodic() {
		SmartDashboard.putNumber("Indexer Current", m_indexerMotor.getOutputCurrent());
		SmartDashboard.putNumber("Indexer Velocity", m_encoder.getVelocity());
	}
}