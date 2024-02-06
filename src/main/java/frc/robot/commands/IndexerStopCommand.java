// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerStopCommand extends Command {
	/** Creates a new IndexerStopCommand. */
	public IndexerStopCommand() {
		addRequirements(IndexerSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		IndexerSubsystem.get().setSpeed(0.0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
