// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ResetToZeroDegreesCommand extends Command {
	private DriveSubsystem m_driveSubsystem;

	public ResetToZeroDegreesCommand() {
		m_driveSubsystem = DriveSubsystem.get();
	}

	@Override
	public void initialize() {
		// m_driveSubsystem.resetHeading();
		m_driveSubsystem.setWheelRotationToZeroDegrees();
	}
}