package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystemAdvanced;

/**
 * The {@code DriveCommand} is responsible for moving the robot from the current
 * pose to a certain target pose.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommandAdvanced extends DriveCommand {

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target.
	 * 
	 * @param targetPose
	 *                          the target pose whose x and y-coordinate values are
	 *                          in meters and yaw value is in degrees
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommandAdvanced(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance,
			PoseEstimationSubsystemAdvanced poseEstimationSubsystem) {
		super(driveSubsystem, () -> {
			var pose = poseEstimationSubsystem.estimatedPose();
			poseEstimationSubsystem.recordPose("Target", targetPose);
			return driveSubsystem.getPose().plus(targetPose.minus(pose));
		}, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target.
	 * 
	 * @param targetPose
	 *                          the target pose whose x and y-coordinate values are
	 *                          in meters and yaw value is in degrees
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommandAdvanced(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance) {
		super(driveSubsystem, targetPose, distanceTolerance, angleTolerance);
	}

	public DriveCommandAdvanced(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			Translation2d translation2d, double distanceToTarget, double distanceTolerance,
			double angleTolerance) {
		super(driveSubsystem, () -> {
			// var pose = poseEstimationSubsystem.estimatedPose();
			// poseEstimationSubsystem.recordPose("Target", targetPose);
			// return driveSubsystem.getPose().plus(targetPose.minus(pose));
			return null;
		}, distanceTolerance, angleTolerance);
	}

	public DriveCommandAdvanced(DriveSubsystem driveSubsystem,
			Translation2d translation2d, double distanceToTarget, double distanceTolerance,
			double angleTolerance) {
		super(driveSubsystem, () -> {
			// var pose = poseEstimationSubsystem.estimatedPose();
			// poseEstimationSubsystem.recordPose("Target", targetPose);
			// return driveSubsystem.getPose().plus(targetPose.minus(pose));
			return null;
		}, distanceTolerance, angleTolerance);
	}

}