// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeCommand extends Command {
	private DriveSubsystem m_driveSubsystem;
	private double m_seconds;
	private final Timer m_timer = new Timer();

	/** Creates a new DriveTimeCommand. */
	public DriveTimeCommand(DriveSubsystem subsystem, double seconds) {
		m_driveSubsystem = subsystem;
		m_seconds = seconds;
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(1, 0, 0), true);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.get() >= m_seconds;
	}
}
