// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AimSpeedCommand extends Command {
	private final ShooterSubsystem m_shooterSubsystem;

	/** Creates a new ClimberMove. */
	public AimSpeedCommand(ShooterSubsystem subsystem) {
		m_shooterSubsystem = subsystem;
		addRequirements(m_shooterSubsystem);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_shooterSubsystem.setSpeed(15);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooterSubsystem.setSpeed(0);
	}
}