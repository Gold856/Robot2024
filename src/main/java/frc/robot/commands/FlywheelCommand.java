// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends Command {
	private Operation m_operation;
	FlywheelSubsystem m_flywheelSubsystem;
	double m_flywheelParam;

	public enum Operation {
		SET_VELOCITY,
		SETTLE;
	}

	/** Creates a new FlywheelCommand. */
	public FlywheelCommand(FlywheelSubsystem flywheelSubsytem, Operation operation, double flywheelParam) {
		m_flywheelSubsystem = flywheelSubsytem;
		m_flywheelParam = flywheelParam;
		m_operation = operation;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_flywheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == Operation.SET_VELOCITY) {
			m_flywheelSubsystem.setVelocity(m_flywheelParam);
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
		if (m_operation == Operation.SET_VELOCITY) {
			return true;
		} else if (m_operation == Operation.SETTLE) {
			return m_flywheelSubsystem.atSetpoint();
		}
		System.out.println("Unreachable code in FlywheelCommand");
		return false; // unreachable code
	}
}
