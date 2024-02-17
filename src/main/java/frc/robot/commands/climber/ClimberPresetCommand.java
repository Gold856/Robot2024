// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPresetCommand extends Command {
	private final ClimberSubsystem m_climberSubsystem;
	private final ClimberOperation m_operation;

	public enum ClimberOperation {
		TOP,
		ZERO
	}

	/** Creates a new ClimberMove. */
	public ClimberPresetCommand(ClimberSubsystem subsystem, ClimberOperation operation) {
		m_climberSubsystem = subsystem;
		m_operation = operation;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == ClimberOperation.TOP) {
			m_climberSubsystem.setPosition(ClimbConstants.kMaxExtension, ClimbConstants.kMaxExtension);
		} else if (m_operation == ClimberOperation.ZERO) {
			m_climberSubsystem.setPosition(0, 0);
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
		return m_climberSubsystem.atleftSetpoint() && m_climberSubsystem.atrightSetpoint();
	}
}