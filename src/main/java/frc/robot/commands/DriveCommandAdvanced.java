package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.common.PoseEstimationSubsystemAdvanced;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem.Pose;

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
	public DriveCommandAdvanced(Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		super(targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to navigate the robot
	 * towards the specified target and stop at the specified distance
	 * away from the target.
	 * 
	 * @param targetPosition    the target position whose x and y-coordinate values
	 *                          are in meters
	 * @param distanceToTarget  the desired distance to the target
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommandAdvanced(Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance,
			double angleTolerance) {
		super(() -> PoseEstimationSubsystem.getTargetPose(DriveSubsystem.get().getPose(), targetPosition,
				distanceToTarget),
				distanceTolerance,
				angleTolerance);
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	protected void recordPose(String entryName, Pose2d value) {
		if (value != null && !(value instanceof Pose))
			value = new Pose(value.getX(), value.getY(), value.getRotation().getDegrees());
		PoseEstimationSubsystemAdvanced.get().record(entryName, value);
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	protected void recordString(String entryName, String value) {
		PoseEstimationSubsystemAdvanced.get().record(entryName, value);
	}

}