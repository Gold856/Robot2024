// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AlignWhileDrivingCommand extends Command {
	Supplier<Double> m_forwardSpeed, m_strafeSpeed, m_goalAngle;
	ClampedController m_controller;
	DriveSubsystem m_driveSubsystem;
	PoseEstimationSubsystem m_PoseEstimationSubsystem;

	/** Creates a new AlignWhileDriving. */
	public AlignWhileDrivingCommand(Supplier<Double> speedX, Supplier<Double> speedY, DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_controller = new ClampedController(0.01, 0.1, 0.3);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double fwdSpeed = -kTeleopMaxSpeed
				* MathUtil.applyDeadband(m_forwardSpeed.get(), ControllerConstants.kDeadzone);
		double strSpeed = -kTeleopMaxSpeed
				* MathUtil.applyDeadband(m_strafeSpeed.get(), ControllerConstants.kDeadzone);
		// TODO
		m_controller.calculate(0);
		// setModuleStates(calculateModuleStates(new ChassisSpeeds(fwdSpeed, strSpeed,
		// rotSpeed), true));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
