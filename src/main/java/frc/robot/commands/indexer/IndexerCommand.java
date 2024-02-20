// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Controls the indexer, sends it forward or in reverse
 * 
 * @author Gabriel West
 * @author Andrew Hwang
 */
public class IndexerCommand extends Command {
	private final double m_speed;
	private final IndexerSubsystem m_indexerSubsystem;

	/**
	 * Runs the indexer forward (towards the shooter)
	 * 
	 * @param indexerSubsystem
	 * @return The indexer full speed forward
	 */
	public static IndexerCommand getFowardCommand(IndexerSubsystem indexerSubsystem) {
		return new IndexerCommand(indexerSubsystem, 1.0);
	}

	/**
	 * Runs the indexer in reverse (towards the intake)
	 * 
	 * @param indexerSubsystem
	 * @return The indexer at full speed in reverse
	 */
	public static IndexerCommand getReverseCommand(IndexerSubsystem indexerSubsystem) {
		return new IndexerCommand(indexerSubsystem, -1.0);
	}

	/** Creates a new IndexerFowardCommand. */
	private IndexerCommand(IndexerSubsystem indexerSubsystem, double speed) {
		m_speed = speed;
		m_indexerSubsystem = indexerSubsystem;
		addRequirements(m_indexerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_indexerSubsystem.setSpeed(m_speed);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
