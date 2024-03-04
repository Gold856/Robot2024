// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimshooter;

import static frc.robot.Constants.AimerConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AimHeightCommand extends Command {
	private AimHeightOperation m_operation;
	// private double m_distanceMeters;
	private Double m_distanceToSpeaker;
	private AimerSubsystem m_aimerSubsystem;
	private Targeter m_targeter;
	private LimeLightSubsystem m_limelightSubsystem;

	public enum AimHeightOperation {
		CALC_AND_SET, // Calculate Angle at current position (changes)
		SET_PRESET_DEFAULT, // For checkout, set shooter down (static)
		PRESET_AMP,
		SET_LOW,
		PRESET_SUBWOOFER,
		HOLD,
		DOWN_ADJUST, // Fine tune down
		UP_ADJUST, // Fine tune up
		SETTLE, // Paired with above in Robot Container
		STOP, // Currently not in use (??)
		SOURCE,
		PRESET_PODIUM
	}

	/** Creates a new AimCommand. */
	public AimHeightCommand(AimerSubsystem subsystem, Targeter targeter, AimHeightOperation operation) {
		m_operation = operation;
		m_aimerSubsystem = subsystem;
		m_targeter = targeter;
		addRequirements(m_aimerSubsystem);
	}

	/** Creates a new AimCommand. */
	public AimHeightCommand(AimerSubsystem subsystem, Targeter targeter, AimHeightOperation operation,
			LimeLightSubsystem limelightSubsystem) {
		m_operation = operation;
		m_aimerSubsystem = subsystem;
		m_targeter = targeter;
		m_limelightSubsystem = limelightSubsystem;
		// m_distanceMeters = poseEstimationSubsystem.distanceTo();
		addRequirements(m_aimerSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_distanceToSpeaker = m_limelightSubsystem.distanceToSpeaker();
		SmartDashboard.putNumber("distance to speaker", m_distanceToSpeaker);
		switch (m_operation) {
			case CALC_AND_SET:
				double actuatorHeightSetpoint = m_targeter.getAngle(m_distanceToSpeaker);
				SmartDashboard.putNumber("Caluclated Height in Command", actuatorHeightSetpoint);
				m_aimerSubsystem.setAimerHeight(actuatorHeightSetpoint);
				break;
			case SET_PRESET_DEFAULT:
				m_aimerSubsystem.setAimerHeight(kDefaultActuatorHeight);
				break;
			case PRESET_SUBWOOFER:
				m_aimerSubsystem.setAimerHeight(kSubwooferActuatorHeight);
				break;
			case PRESET_AMP:
				m_aimerSubsystem.setAimerHeight(kAmpActuatorHeight);
				break;
			case SOURCE:
				// TODO add source constant
				m_aimerSubsystem.setAimerHeight(kDefaultActuatorHeight);
				break;
			case SET_LOW:
				m_aimerSubsystem.setAimerHeight(0.2);
				break;
			case HOLD:
				m_aimerSubsystem.setAimerHeight(m_aimerSubsystem.getAimerHeight());
				break;
			case STOP:
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