// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMove extends Command {
	private final ClimberSubsystem m_climberSubsystem;
	private double m_speed;

	/** Creates a new ClimberMove. */
	public ClimberMove(ClimberSubsystem subsystem, double speed) {
		m_climberSubsystem = subsystem;
		m_speed = speed;
		addRequirements(ClimberSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_climberSubsystem.setSpeed(m_speed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_climberSubsystem.setSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
