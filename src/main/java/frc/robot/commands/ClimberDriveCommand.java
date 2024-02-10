// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDriveCommand extends Command {

	private final ClimberSubsystem m_climberSubsystem;
	private final Supplier<Double> m_left;
	private final Supplier<Double> m_right;
	private final double m_speed;

	/** Creates a new LeftClimberDrive. */
	public ClimberDriveCommand(ClimberSubsystem subsystem, Supplier<Double> left, Supplier<Double> right,
			double speed) {
		m_left = left;
		m_right = right;
		m_speed = speed;
		m_climberSubsystem = subsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_climberSubsystem.setPosition(m_left.get(), m_right.get());
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
		if (m_climberSubsystem.atleftSetpoint() && m_climberSubsystem.atrightSetpoint()) {
			return true;
		}
		return false;
	}
}
