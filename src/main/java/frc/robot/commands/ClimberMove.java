// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMove extends Command {
	private final ClimberSubsystem m_climberSubsystem;

	public enum Operation {
		ZERO,
		MID,
		TOP,
		STOP
	}

	private final Operation m_operation;

	/** Creates a new ClimberMove. */
	public ClimberMove(ClimberSubsystem subsystem, Operation operation) {
		m_climberSubsystem = subsystem;
		m_operation = operation;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == Operation.ZERO) {
			m_climberSubsystem.setPosition(0, 0);
		} else if (m_operation == Operation.MID) {
			m_climberSubsystem.setPosition(50, 50);
		} else if (m_operation == Operation.TOP) {
			m_climberSubsystem.setPosition(100, 100);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		switch (m_operation) {
			case ZERO, MID, TOP:
				return m_climberSubsystem.atleftSetpoint() && m_climberSubsystem.atrightSetpoint();
			case STOP:
				return true;
			default:
				return false;
		}
	}
}