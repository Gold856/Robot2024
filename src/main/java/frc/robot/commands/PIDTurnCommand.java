// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class PIDTurnCommand extends Command {
	private final double m_targetAngle;
	private final double m_angleThreshold;
	private final DriveSubsystem m_driveSubsystem;
	private final PIDController m_controller = new PIDController(0.01, 0.005, 0);

	/**
	 * Creates a new {@code PIDTurnCommand} with the given params.
	 * 
	 * @param targetAngle    the angle to turn to (0 is away from you)
	 * @param angleThreshold the threshold
	 */
	public PIDTurnCommand(DriveSubsystem driveSubsystem, double targetAngle, double angleThreshold) {
		m_targetAngle = targetAngle;
		m_angleThreshold = angleThreshold;
		m_driveSubsystem = driveSubsystem;
		m_controller.enableContinuousInput(0, 360);
		m_controller.setSetpoint(m_targetAngle);
		m_controller.setTolerance(m_angleThreshold);
		m_controller.setIZone(10);
		addRequirements(driveSubsystem);
	}

	/**
	 * Called when this command is first scheduled
	 */
	@Override
	public void initialize() {
	}

	/**
	 * Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() {
		double rot = m_driveSubsystem.getHeading().getDegrees();
		double speed = m_controller.calculate(rot);

		m_driveSubsystem.setModuleStates(0, 0, speed, false);
		SmartDashboard.putNumber("Heading", rot);
		SmartDashboard.putNumber("Target", m_targetAngle);
		SmartDashboard.putNumber("Rotation speed", speed);
	}

	/**
	 * Checks if the turn command should be completed
	 * 
	 * @return {@code true} if the robot has reached its goal;
	 *         {@code false} otherwise.
	 */
	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}
