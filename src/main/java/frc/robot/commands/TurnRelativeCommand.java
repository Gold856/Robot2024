// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRelativeCommand extends Command {
	private DriveSubsystem m_driveSubsystem;
	private double m_angle;
	private PIDController m_controller;

	/** Creates a new TurnRelativeaCommand. */
	public TurnRelativeCommand(DriveSubsystem subsystem, double angle) {
		m_driveSubsystem = subsystem;
		m_angle = angle;
		m_controller = new PIDController(0.0075, 0, 0);
		m_controller.enableContinuousInput(0, 360);
		m_controller.setTolerance(5);
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_controller.setSetpoint(m_driveSubsystem.getHeading().getDegrees() + m_angle);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		var moduleStates = m_driveSubsystem.calculateModuleStates(
				new ChassisSpeeds(0, 0, m_controller.calculate(m_driveSubsystem.getHeading().getDegrees())), false);
		m_driveSubsystem.setSwerveStates(moduleStates);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}
}
