// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimerDriveCommand extends Command {

	private final ShooterSubsystem m_shooterSubsystem;
	private final Supplier<Double> m_speed;

	/** Creates a new AimerDriveCommand. */
	public AimerDriveCommand(ShooterSubsystem subsystem, Supplier<Double> speed) {
		m_shooterSubsystem = subsystem;
		m_speed = speed;
		addRequirements(m_shooterSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(m_speed.get(), ControllerConstants.kDeadzone) * 0.4;
		if (speed != 0) {
			m_shooterSubsystem.setManual(true); // Joystick control
			// Prevent lead screw from going out of bounds
			if ((m_shooterSubsystem.getActuatorHeight() > ShooterConstants.kShooterMaxEncoderValue) && (speed > 0)) {
				speed = 0;
			} else if ((m_shooterSubsystem.getActuatorHeight() <= 0.05) && (speed < 0)) {
				speed = 0;
			}
			m_shooterSubsystem.setSpeed(speed);
		} else {
			m_shooterSubsystem.setManual(false); // Setpoint control
			m_shooterSubsystem.setActuatorHeight(m_shooterSubsystem.getActuatorHeight());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooterSubsystem.stopMotor();
	}
}