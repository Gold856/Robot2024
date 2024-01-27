// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;

public class LEDCommand extends Command {
	private StatusCode m_StatusCode;

	/** Creates a new LEDCommand. */
	public LEDCommand(StatusCode code) {
		m_StatusCode = code;
		addRequirements(ArduinoSubsystem.get());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ArduinoSubsystem.get().setCode(m_StatusCode);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}