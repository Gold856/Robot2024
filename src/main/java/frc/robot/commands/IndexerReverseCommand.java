// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerReverseCommand extends Command {
	private final double m_speed;

	/** Creates a new IndexerFowardCommand. */
	public IndexerReverseCommand(double speed) {
		m_speed = -speed;
		addRequirements(IndexerSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		IndexerSubsystem.get().setSpeed(m_speed);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
