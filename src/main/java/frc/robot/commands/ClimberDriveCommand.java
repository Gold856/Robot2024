// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDriveCommand extends Command {

	private final ClimberSubsystem m_climberSubsystem;
	private final Supplier<Double> m_left;
	private final Supplier<Double> m_right;

	/** Creates a new LeftClimberDrive. */
	public ClimberDriveCommand(ClimberSubsystem subsystem, Supplier<Double> left, Supplier<Double> right) {
		m_left = left;
		m_right = right;
		m_climberSubsystem = subsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_climberSubsystem.setPosition(
				Math.abs(MathUtil.applyDeadband(Math.min(m_left.get(), 0), ControllerConstants.kDeadzone)
						* ClimbConstants.kMaxExtension),
				Math.abs(MathUtil.applyDeadband(Math.min(m_right.get(), 0), ControllerConstants.kDeadzone)
						* ClimbConstants.kMaxExtension));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
}
