// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
	private CANSparkMax m_indexerMotor;

	/**
	 * Creates a new IndexerSubsystem.
	 * 
	 * @throws Exception
	 */
	public IndexerSubsystem() {

		m_indexerMotor = new CANSparkMax(IndexerConstants.kIndexerPort, IndexerConstants.kIndexerMotorType);
		m_indexerMotor.setIdleMode(IdleMode.kBrake);
		m_indexerMotor.enableVoltageCompensation(12);
		m_indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerSmartCurrentLimit);
		m_indexerMotor.setSecondaryCurrentLimit(IndexerConstants.kIndexerPeakCurrentLimit);
	}

	public void periodic() {
	}

	public void setSpeed(double speed) {
		m_indexerMotor
				.set(speed);
	}

	public void stop() {
		setSpeed(0.0);
	}
}
