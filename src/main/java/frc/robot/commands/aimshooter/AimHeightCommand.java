// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimshooter;

import static frc.robot.Constants.AimerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter;
import frc.robot.subsystems.AimerSubsystem;

public class AimHeightCommand extends Command {
	private AimHeightOperation m_operation;
	private double m_distanceMeters;
	private AimerSubsystem m_aimerSubsystem;
	private Targeter m_targeter;

	public enum AimHeightOperation {
		CALC_AND_SET, // Calculate Angle at current position (changes)
		SET_PRESET_DEFAULT, // For checkout, set shooter down (static)
		PRESET_AMP,
		PRESET_SUBWOOFER,
		HOLD,
		DOWN_ADJUST, // Fine tune down
		UP_ADJUST, // Fine tune up
		SETTLE, // Paired with above in Robot Container
		STOP // Currently not in use (??)
	}

	/** Creates a new AimCommand. */
	public AimHeightCommand(AimerSubsystem subsystem, Targeter targeter, AimHeightOperation operation) {
		m_operation = operation;
		m_aimerSubsystem = subsystem;
		m_targeter = targeter;
		addRequirements(m_aimerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		switch (m_operation) {
			case CALC_AND_SET:
				double actuatorHeightSetpoint = m_targeter.calcAimerHeightFromDistance(m_distanceMeters);
				m_aimerSubsystem.setAimerHeight(actuatorHeightSetpoint);
				break;
			case SET_PRESET_DEFAULT:
				m_aimerSubsystem.setAimerHeight(kDefaultActuatorHeight);
				break;
			case UP_ADJUST:
				m_aimerSubsystem.adjustAimerSetpoint(kAdjustAmount);
				break;
			case DOWN_ADJUST:
				m_aimerSubsystem.adjustAimerSetpoint(-kAdjustAmount);
				break;
			case HOLD:
				m_aimerSubsystem.setAimerHeight(m_aimerSubsystem.getAimerHeight());
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
		if (m_operation == AimHeightOperation.SETTLE) {
			return m_aimerSubsystem.atAimerSetpoint();
		}
		return true;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (m_operation == AimHeightOperation.STOP) {
			m_aimerSubsystem.stopMotor();
		}
	}
}