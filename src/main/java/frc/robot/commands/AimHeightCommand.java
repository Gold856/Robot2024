// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter.PhysicsAndMathTargeter;
import frc.robot.subsystems.ShooterSubsystem;

public class AimHeightCommand extends Command {
	private AimHeightOperation m_operation;
	private double m_distanceMeters;
	private ShooterSubsystem m_shooterSubsystem;
	private PhysicsAndMathTargeter m_targeter;

	public enum AimHeightOperation {
		CMD_CALC_AND_SET, // Calculate Angle at current position (changes)
		CMD_SET_PRESET_DEFAULT, // For checkout, set shooter down (static)
		CMD_PRESET_AMP,
		CMD_PRESET_SUBWOOFER,
		CMD_HOLD,
		CMD_DOWN_ADJUST, // Fine tune down
		CMD_UP_ADJUST, // Fine tune up
		CMD_SETTLE, // Paired with above in Robot Container
		CMD_STOP // Currently not in use (??)
	}

	/** Creates a new AimCommand. */
	public AimHeightCommand(ShooterSubsystem subsystem, PhysicsAndMathTargeter targeter, AimHeightOperation operation) {
		m_operation = operation;
		m_shooterSubsystem = subsystem;
		m_targeter = targeter;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_shooterSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		switch (m_operation) {
			case CMD_CALC_AND_SET:
				double actuatorHeightSetpoint = m_targeter.calcActuatorHeightFromDistance(m_distanceMeters);
				m_shooterSubsystem.setActuatorHeight(actuatorHeightSetpoint);
				break;
			case CMD_SET_PRESET_DEFAULT:
				m_shooterSubsystem.setActuatorHeight(kDefaultActuatorHeight);
				break;
			case CMD_UP_ADJUST:
				m_shooterSubsystem.adjustActuatorSetpoint(kAdjustAmount);
				break;
			case CMD_DOWN_ADJUST:
				m_shooterSubsystem.adjustActuatorSetpoint(-kAdjustAmount);
				break;
			case CMD_HOLD:
				m_shooterSubsystem.setActuatorHeight(m_shooterSubsystem.getActuatorHeight());
				break;
			default:
				break;
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_operation == AimHeightOperation.CMD_SETTLE) {
			return m_shooterSubsystem.atActuatorSetpoint();
		}
		return true;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (m_operation == AimHeightOperation.CMD_STOP) {
			m_shooterSubsystem.stopMotor();
		}
	}
}