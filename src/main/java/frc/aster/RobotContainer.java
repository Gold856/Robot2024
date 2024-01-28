// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.aster.subsystems.DriveSubsystem;
import frc.common.PoseEstimationSubsystem;
import frc.aster.Constants.ControllerConstants;
import frc.aster.Constants.ControllerConstants.Button;
import frc.aster.commands.drive.DefaultDriveCommand;
import frc.aster.commands.drive.DriveDistanceCommand;
import hlib.drive.Pose;
import hlib.drive.Position;

public class RobotContainer implements frc.common.RobotContainer {
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem();

	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);

	public RobotContainer() {
		m_poseEstimationSubsystem.addPoseSupplier("Pose2D@Odometry",
				() -> PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose()));
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		m_controller.button(Button.kSquare).whileTrue(new DriveDistanceCommand(2.0, 0.1));
		m_controller.button(Button.kX).whileTrue(new DriveDistanceCommand(-2.0, 0.1));
		m_controller.button(Button.kCircle).whileTrue(new TurnCommand(30, 2));
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
		m_controller.button(Button.kTriangle).whileTrue(new TurnCommand(turnSupplier, 2)
				.andThen(new DriveDistanceCommand(driveSupplier, 0.1)));
		m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				() -> -m_controller.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kLeftTrigger),
				() -> m_controller.getRawAxis(ControllerConstants.Axis.kRightTrigger)));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}