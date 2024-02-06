// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

	private CANSparkMax m_indexerMotor;
	private static IndexerSubsystem instance;

	/**
	 * Creates a new IndexerSubsystem.
	 * 
	 * @throws Exception
	 */
	public IndexerSubsystem() {
		if (instance != null) {
			try {
				throw new Exception("Indexer subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		instance = this;

		m_indexerMotor = new CANSparkMax(IndexerConstants.kIndexerPort, IndexerConstants.kIndexerMotorType);
		m_indexerMotor.setIdleMode(IdleMode.kBrake);
		m_indexerMotor.enableVoltageCompensation(12);
		m_indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerSmartCurrentLimit);
		m_indexerMotor.setSecondaryCurrentLimit(IndexerConstants.kIndexerPeakCurrentLimit);
	}

	public static IndexerSubsystem get() {
		return instance;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("alex wantes pneumatics so we will steal it from him; also motor temp: ",
				m_indexerMotor.getMotorTemperature());
	}

	public void setSpeed(double speed) {
		m_indexerMotor
				.set(Math.min(IndexerConstants.kIndexerMaxSpeed, Math.max(IndexerConstants.kIndexerMinSpeed, speed)));
	}

	public void stop() {
		setSpeed(0.0);
	}

}
