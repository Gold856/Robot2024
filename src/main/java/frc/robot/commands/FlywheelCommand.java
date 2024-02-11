// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends Command {
	private final Operation m_operation;
	private final FlywheelSubsystem m_flywheelSubsystem;
	private final double m_rpm;

	public enum Operation {
		/** Set the velocity and end immediately. */
		SET_VELOCITY,
		/** Set the velocity and end when at setpoint. */
		SETTLE;
	}

	/**
	 * Creates a new FlywheelCommand.
	 * 
	 * @param flywheelSubsytem The subsystem.
	 * @param operation        The operation.
	 * @param rpm              The RPM to spin the motor at.
	 */
	public FlywheelCommand(FlywheelSubsystem flywheelSubsytem, Operation operation, double rpm) {
		m_flywheelSubsystem = flywheelSubsytem;
		m_rpm = rpm;
		m_operation = operation;
		addRequirements(m_flywheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_operation == Operation.SET_VELOCITY) {
			m_flywheelSubsystem.setVelocity(m_rpm);
		}
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
