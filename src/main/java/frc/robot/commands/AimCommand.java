package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Targeter;
import frc.robot.subsystems.ShooterSubsystem;

public class AimCommand extends Command {
	private Operation m_operation;
	private double m_distanceMeters;
	private ShooterSubsystem m_shooterSubsystem;
	private Targeter m_targeter;
	private double m_kDefaultActuatorHeight = ShooterConstants.kDefaultActuatorHeight;

	public enum Operation {
		CMD_UP,
		CMD_DOWN,
		CMD_SETTLE,
		CMD_STOP
	}

	/** Creates a new AimCommand. */
	public AimCommand(ShooterSubsystem subsystem, Targeter targeter, Operation operation) {
		m_operation = operation;
		// m_distanceMeters = SimpleVision.getDistance(); //TODO fix vision
		m_shooterSubsystem = subsystem;
		m_targeter = targeter;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_shooterSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == Operation.CMD_UP) {
			double actuatorHeightSetpoint = m_targeter.calcActuatorHeightFromAngle(m_distanceMeters);
			m_shooterSubsystem.setActuatorHeight(actuatorHeightSetpoint);
		} else if (m_operation == Operation.CMD_DOWN) {
			m_shooterSubsystem.setActuatorHeight(m_kDefaultActuatorHeight);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {

		if (m_operation == Operation.CMD_UP) {
			return true;
		} else if (m_operation == Operation.CMD_SETTLE) {
			return m_shooterSubsystem.atActuatorSetpoint();
		} else if (m_operation == Operation.CMD_STOP) {
			return true;
		}
		return true;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooterSubsystem.stopMotors();
	}

}