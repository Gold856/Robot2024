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
		SETTLE,
		REVERSE;
	}

	/** Creates a new FlywheelCommand. */
	public FlywheelCommand(Operation operation, double flywheelParam) {
		m_flywheelParam = flywheelParam;
		m_operation = operation;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(FlywheelSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == Operation.SET_VELOCITY) {
			FlywheelSubsystem.get().setVelocity(m_flywheelParam);
		} else if (m_operation == Operation.REVERSE) {
			FlywheelSubsystem.get().setVelocityForNegatives();
			FlywheelSubsystem.get().setSpeed(-0.5);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (m_operation == Operation.REVERSE) {
			FlywheelSubsystem.get().setSpeed(0);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_operation == Operation.SET_VELOCITY) {
			return true;
		} else if (m_operation == Operation.SETTLE) {
			return FlywheelSubsystem.get().atSetpoint();
		} else if (m_operation == Operation.REVERSE) {
			return false;
		}
		return true; // TODO: This is robot 2022 code, why does it return true??
	}
}
