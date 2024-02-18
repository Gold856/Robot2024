// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArduinoSubsystem;

public class TimedLEDCommand extends Command {
	private ArduinoSubsystem m_ArduinoSubsystem;
	private double m_seconds;
	private final Timer m_timer = new Timer();
	private ArduinoSubsystem.StatusCode m_statusCode;

	/**
	 * Creates a new TimedLCommand.
	 * 
	 * @param subsystem The subsystem
	 * @param seconds   Time to drive in seconds
	 * @param speed     The speed in percent output
	 */
	public TimedLEDCommand(ArduinoSubsystem subsystem, double seconds, ArduinoSubsystem.StatusCode statusCode) {
		m_ArduinoSubsystem = subsystem;
		m_seconds = seconds;
		m_statusCode = statusCode;
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
		m_ArduinoSubsystem.setLEDs(m_statusCode);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_ArduinoSubsystem.setCode(ArduinoSubsystem.StatusCode.DEFAULT);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.get() >= m_seconds;
	}
}
