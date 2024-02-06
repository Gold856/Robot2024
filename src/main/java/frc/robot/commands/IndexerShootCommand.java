// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerShootCommand extends Command {
	private double m_time;
	private double m_speed;
	private Timer m_timer;

	/** Creates a new IndexerShootCommand. */
	public IndexerShootCommand(double timeSeconds, double speed) {
		m_timer = new Timer();
		m_time = timeSeconds;
		m_speed = speed;
		addRequirements(IndexerSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.start();
		IndexerSubsystem.get().setSpeed(m_speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		IndexerSubsystem.get().stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(m_time);
	}
}
