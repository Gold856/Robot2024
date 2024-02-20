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

/**
 * The presets for the climber
 * 
 * @author Dominick Marrello
 */
public class ClimberPresetCommand extends Command {
	private final ClimberSubsystem m_climberSubsystem;
	private final ClimberOperation m_operation;
	private final Supplier<Double> m_left;
	private final Supplier<Double> m_right;
	private double leftJoystick;
	private double rightJoystick;
	private boolean end = false;

	public enum ClimberOperation {
		TOP,
		ZERO
	}

	/** Creates a new ClimberMove. */
	public ClimberPresetCommand(ClimberSubsystem subsystem, ClimberOperation operation, Supplier<Double> left,
			Supplier<Double> right) {
		m_climberSubsystem = subsystem;
		m_operation = operation;
		m_left = left;
		m_right = right;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		end = false;
		if (m_operation == ClimberOperation.TOP) { // the top preset
			m_climberSubsystem.setPosition(ClimbConstants.kMaxExtension, ClimbConstants.kMaxExtension);
		} else if (m_operation == ClimberOperation.ZERO) { // the bottom preset
			m_climberSubsystem.setPosition(0, 0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		leftJoystick = MathUtil.applyDeadband(m_left.get(), ControllerConstants.kDeadzone);
		rightJoystick = MathUtil.applyDeadband(m_right.get(), ControllerConstants.kDeadzone);

		if (leftJoystick != 0 || rightJoystick != 0) { // if the joysticks are being used it is the end of the command
			end = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_climberSubsystem.setSpeed(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (m_climberSubsystem.atleftSetpoint() && m_climberSubsystem.atrightSetpoint()) || end;
	}
}