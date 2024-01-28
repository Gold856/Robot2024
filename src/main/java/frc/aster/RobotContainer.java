// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.aster.subsystems.DriveSubsystem;
import frc.common.PoseEstimationSubsystem;
import frc.aster.Constants.ControllerConstants;
import frc.aster.commands.drive.DefaultDriveCommand;
import frc.aster.commands.drive.DriveDistanceCommand;
import hlib.drive.Pose;

import hlib.drive.Position;

public class RobotContainer implements frc.common.RobotContainer {
	DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem();

	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);
	// private final Joystick m_operatorController = new
	// Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		m_poseEstimationSubsystem.addPoseSupplier("odometry",
				() -> PoseEstimationSubsystem.toPose(DriveSubsystem.get().getPose()));
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		new JoystickButton(m_driverController, ControllerConstants.Button.kSquare)
				.whileTrue(new DriveDistanceCommand(2.0, 0.1));
		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.whileTrue(new DriveDistanceCommand(-2.0, 0.1));
		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.whileTrue(new TurnCommand(30, 2));
		var target = new Position(6.809, -3.859);
		Supplier<Double> turnSupplier = () -> {
			Pose pose = m_poseEstimationSubsystem.estimatedPose();
			if (pose != null)
				return pose.displacementTo(target).angleInDegrees() - pose.yawInDegrees();
			return 0.0;
		};
		Supplier<Double> driveSupplier = () -> {
			Pose pose = m_poseEstimationSubsystem.estimatedPose();
			if (pose != null)
				return 1 - pose.distance(target);
			return 0.0;
		};
		// new JoystickButton(m_operatorController,
		// ControllerConstants.Button.kTriangle)
		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
				.whileTrue(new TurnCommand(turnSupplier, 2)
						.andThen(new DriveDistanceCommand(driveSupplier, 0.1)));

		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger)));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
