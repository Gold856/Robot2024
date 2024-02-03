package frc.robot.commands;

import java.util.function.Supplier;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

/**
 * The {@code DriveCommand} is responsible for moving the robot from the current
 * pose to a certain target pose.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand extends Command {

	/**
	 * The
	 * {@code Supplier<Pose2d>} that calculates the target pose to which the robot
	 * should move.
	 * This is used at the commencement of this {@code DriveCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	private Supplier<Pose2d> m_targetPoseCalculator;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerX;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the y
	 * dimension in meters.
	 */
	private ProfiledPIDController m_controllerY;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in angles.
	 */
	private ProfiledPIDController m_controllerYaw;

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
	public DriveCommand(Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		this(() -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to navigate the robot
	 * towards the specified target and stop at the specified distance
	 * away from the target.
	 * 
	 * @param currentPose       the current {@code Pose2d} of the robot
	 * @param targetPosition    the target position whose x and y-coordinate values
	 *                          are in meters
	 * @param distanceToTarget  the desired distance to the target
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommand(Pose2d currentPose, Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance,
			double angleTolerance) {
		this(() -> PoseEstimationSubsystem.getTargetPose(currentPose, targetPosition, distanceToTarget),
				distanceTolerance,
				angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param targetPoseCalculator
	 *                             a {@code Supplier<Pose2d>} that calculates the
	 *                             target pose to which the robot should move.
	 *                             This is used at the commencement of this {@code
	 *                             DriveCommand} (i.e.,
	 *                             when the scheduler begins to periodically execute
	 *                             this {@code
	 *                             DriveCommand})
	 * 
	 * @param distanceTolerance    the distance error in
	 *                             meters which is
	 *                             tolerable
	 * @param angleTolerance
	 *                             the angle error in degrees which is tolerable
	 */
	public DriveCommand(Supplier<Pose2d> targetPoseCalculator, double distanceTolerance, double angleTolerance) {
		m_targetPoseCalculator = targetPoseCalculator;
		var constraints = new TrapezoidProfile.Constraints(Constants.DriveConstants.kDriveMaxVelocity,
				Constants.DriveConstants.kDriveMaxAcceleration);
		m_controllerX = new ProfiledPIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI,
				Constants.DriveConstants.kDriveD, constraints);
		m_controllerY = new ProfiledPIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI,
				Constants.DriveConstants.kDriveD, constraints);
		m_controllerYaw = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
				DriveConstants.kTurnD,
				new TrapezoidProfile.Constraints(Constants.DriveConstants.kTurnMaxVelocity,
						Constants.DriveConstants.kTurnMaxAcceleration));
		m_controllerX.setTolerance(distanceTolerance);
		m_controllerY.setTolerance(distanceTolerance);
		m_controllerYaw.setTolerance(angleTolerance);
		m_controllerYaw.enableContinuousInput(-180, 180);
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this {@code DriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = DriveSubsystem.get().getPose();
		var targetPose = pose;
		try {
			targetPose = m_targetPoseCalculator.get();
		} catch (Exception e) {
		}
		m_controllerX.reset(pose.getX());
		m_controllerY.reset(pose.getY());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		m_controllerX.setGoal(targetPose.getX());
		m_controllerY.setGoal(targetPose.getY());
		m_controllerYaw.setGoal(targetPose.getRotation().getDegrees());
		SmartDashboard.putString(
				"drive",
				String.format(
						"drive: initialize - current pose: %s, target pose: %s",
						pose,
						targetPose));
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = DriveSubsystem.get().getPose();
		double speedX = m_controllerX.calculate(pose.getX());
		double speedY = m_controllerY.calculate(pose.getY());
		double speedYaw = m_controllerYaw.calculate(pose.getRotation().getDegrees());
		DriveSubsystem.get().setModuleStates(frc.common.MathUtil.applyThreshold(speedX, DriveConstants.kMinSpeed),
				frc.common.MathUtil.applyThreshold(speedY, DriveConstants.kMinSpeed), speedYaw, true);
		SmartDashboard.putString(
				"drive",
				String.format(
						"distance: execute - velocities in x, y, and yaw dimensions: [%.1f, %.1f, %.1f]",
						speedX, speedY, speedYaw));
	}

	/**
	 * Is invoked once this {@code DriveCommand} is either ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code DriveCommand} was
	 *                    interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().setModuleStates(0, 0, 0, true);
		SmartDashboard.putString("drive",
				"distance: end - : " + (interrupted ? "interrupted"
						: "completed" + String.format(
								"drive: initialize - current pose: %s, target: [%.1f, %.1f, %.1f]",
								"" + DriveSubsystem.get().getPose(),
								m_controllerX.getGoal().position, m_controllerY.getGoal().position,
								m_controllerYaw.getGoal().position)));
	}

	/**
	 * Determines whether or not this {@code DriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerYaw.atGoal();
	}

	/**
	 * Constructs a {@code SequentialCommandGroup} for passing through all of the
	 * specified
	 * {@code Pose2d}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 * @param poses             the {@code Pose2d}s to pass through
	 * @return a {@code SequentialCommandGroup} for passing through all of the
	 *         specified {@code Pose2d}s
	 */
	public static Command createCommand(double distanceTolerance, double angleTolerance, Pose2d... poses) {
		if (poses == null || poses.length == 0)
			return new Command() {
				public boolean isFinished() {
					return true;
				}
			};
		Command c = null;
		for (var pose : poses) {
			c = c == null ? new DriveCommand(() -> pose, distanceTolerance, angleTolerance)
					: c.andThen(new DriveCommand(() -> pose, distanceTolerance, angleTolerance));
		}
		return c;
	}

}