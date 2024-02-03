// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.aster;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.commands.drive.TurnCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.aster.subsystems.DriveSubsystem;
import frc.common.PoseEstimationSubsystemAdvanced;
import frc.aster.Constants.ControllerConstants;
import frc.aster.Constants.ControllerConstants.Button;
import frc.aster.commands.drive.DefaultDriveCommand;
import frc.aster.commands.drive.DriveDistanceCommand;

public class RobotContainer implements frc.common.RobotContainer {
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final PoseEstimationSubsystemAdvanced m_poseEstimationSubsystem = new PoseEstimationSubsystemAdvanced();
	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);

	public RobotContainer() {
		m_poseEstimationSubsystem.addPoseSupplier("Pose2D@Odometry",
				() -> m_driveSubsystem.getPose());
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		m_controller.button(Button.kSquare).whileTrue(new DriveDistanceCommand(2.0, 0.1));
		m_controller.button(Button.kX).whileTrue(new DriveDistanceCommand(-2.0, 0.1));
		m_controller.button(Button.kCircle).whileTrue(new TurnCommand(30, 2));
		// var target = new Translation2d(6.809, -3.859);
		Supplier<Double> turnSupplier = () -> {
			Map<Integer, Rotation2d> m = m_poseEstimationSubsystem.getRotationsToDetectedTags();
			if (m.size() > 0)
				return m.values().iterator().next().getDegrees();
			return 0.0;
		};
		Supplier<Double> driveSupplier = () -> {
			Map<Integer, Double> m = m_poseEstimationSubsystem.getDistancesToDetectedTags();
			if (m.size() > 0)
				return 1 - m.values().iterator().next();
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