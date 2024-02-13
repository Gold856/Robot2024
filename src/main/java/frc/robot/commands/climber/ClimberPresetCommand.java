// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPresetCommand extends Command {
	private final ClimberSubsystem m_climberSubsystem;
	private final Supplier<Double> m_left;
	private final Supplier<Double> m_right;

	/** Creates a new ClimberMove. */
	public ClimberPresetCommand(ClimberSubsystem subsystem, Supplier<Double> left, Supplier<Double> right) {
		m_left = left;
		m_right = right;
		m_climberSubsystem = subsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (m_climberSubsystem.atZero()) {
			m_climberSubsystem.setPosition(ClimbConstants.kMaxExtension, ClimbConstants.kMaxExtension);
		} else {
			m_climberSubsystem.setPosition(0, 0);
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
		if (MathUtil.applyDeadband(m_left.get(), ControllerConstants.kDeadzone) == 0
				|| MathUtil.applyDeadband(m_right.get(), ControllerConstants.kDeadzone) == 0) {
			return true;
		} else {
			return false;
		}
	}
}