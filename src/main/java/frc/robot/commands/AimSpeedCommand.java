// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AimSpeedCommand extends Command {
	private AimSpeedOperation m_operation;
	private double m_distanceMeters;
	private ShooterSubsystem m_shooterSubsystem;

	public enum AimSpeedOperation {
		CMD_ON,
		CMD_OFF
	}

	/** Creates a new AimCommand. */
	public AimSpeedCommand(ShooterSubsystem subsystem, AimSpeedOperation operation) {
		m_operation = operation;
		// m_distanceMeters = SimpleVision.getDistance(); //TODO fix vision
		m_shooterSubsystem = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_shooterSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		switch (m_operation) {
			case CMD_ON:
				m_shooterSubsystem.setSpeed(15);
				break;
			case CMD_OFF:
				m_shooterSubsystem.setSpeed(0);
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

		if (m_operation == AimSpeedOperation.CMD_CALC_AND_SET
				|| m_operation == AimSpeedOperation.CMD_SET_PRESET_DEFAULT) {
			return true;
		} else if (m_operation == AimSpeedOperation.CMD_UP_ADJUST || m_operation == AimSpeedOperation.CMD_DOWN_ADJUST) {
			return true;
		} else if (m_operation == AimSpeedOperation.CMD_SETTLE) {
			return m_shooterSubsystem.atActuatorSetpoint();
		} else if (m_operation == AimSpeedOperation.CMD_STOP) {
			return true;
		}
		return true;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (m_operation == AimSpeedOperation.CMD_STOP) {
			m_shooterSubsystem.stopMotors();
		}
		// m_shooterSubsystem.stopMotors();
	}

}