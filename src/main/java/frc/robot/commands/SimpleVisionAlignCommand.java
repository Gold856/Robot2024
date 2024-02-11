// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

public class SimpleVisionAlignCommand extends Command {
	private DriveSubsystem m_driveSubsystem;
	private SimpleVisionSubsystem m_visionSubsystem;
	private PIDController m_controller;

	/**
	 * Creates a new TurnRelativeCommand.
	 * 
	 * @param subsystem The drive subsystem.
	 * @param angle     The angle in degrees.
	 */
	public SimpleVisionAlignCommand(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_visionSubsystem = visionSubsystem;
		m_controller = new PIDController(0.0075, 0, 0);
		m_controller.enableContinuousInput(0, 360);
		m_controller.setTolerance(5);
		addRequirements(m_driveSubsystem, m_visionSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_controller.setSetpoint(0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double currentAngle = m_visionSubsystem.getAngle();
		double power = m_controller.calculate(currentAngle);
		double powerLimit = .1;
		power = MathUtil.clamp(power, -powerLimit, powerLimit);
		m_driveSubsystem.setModuleStates(0, 0, power, false);
		SmartDashboard.putNumber("error", m_controller.getPositionError());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}
}
