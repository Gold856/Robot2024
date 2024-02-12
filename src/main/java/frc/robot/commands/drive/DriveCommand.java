package frc.robot.commands.drive;

import java.util.function.Supplier;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

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
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier<Pose2d>} that calculates the target pose to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	private Supplier<Pose2d> m_targetPoseSupplier;

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
	 * @param driveSubsystem    the {@code DriveSubsystem} to use
	 * @param targetPose
	 *                          the target pose whose x and y-coordinate values are
	 *                          in meters and yaw value is in degrees
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem     the {@code DriveSubsystem} to use
	 * @param targetPoseSupplier
	 *                           a {@code Supplier<Pose2d>} that provides the
	 *                           target pose to which the robot should move.
	 *                           This is used at the commencement of this {@code
	 *                             DriveCommand} (i.e.,
	 *                           when the scheduler begins to periodically execute
	 *                           this {@code
	 *                             DriveCommand})
	 * 
	 * @param distanceTolerance  the distance error in
	 *                           meters which is
	 *                           tolerable
	 * @param angleTolerance
	 *                           the angle error in degrees which is tolerable
	 */
	public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPoseSupplier, double distanceTolerance,
			double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_targetPoseSupplier = targetPoseSupplier;
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
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this {@code DriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_driveSubsystem.getCorrectedPose();
		var targetPose = pose;
		try {
			targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
		}
		m_controllerX.reset(pose.getX());
		m_controllerY.reset(pose.getY());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		m_controllerX.setGoal(targetPose.getX());
		m_controllerY.setGoal(targetPose.getY());
		m_controllerYaw.setGoal(targetPose.getRotation().getDegrees());

		recordPose("Target@Odometry", targetPose);
		recordString(
				"drive",
				String.format(
						"initialize - current pose: %s, target pose: %s", toString(pose), toString(targetPose)));
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = m_driveSubsystem.getCorrectedPose();
		double speedX = m_controllerX.calculate(pose.getX());
		double speedY = m_controllerY.calculate(pose.getY());
		double speedYaw = m_controllerYaw.calculate(pose.getRotation().getDegrees());
		// speedX = applyThreshold(speedX, DriveConstants.kMinSpeed);
		// speedY = applyThreshold(speedY, DriveConstants.kMinSpeed);
		m_driveSubsystem.setModuleStates(speedX,
				speedY, speedYaw, true);
		recordString(
				"drive", "execute - velocities :" + toString(speedX, speedY, speedYaw) + ", pose: "
						+ toString(m_driveSubsystem.getCorrectedPose()));
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
		m_driveSubsystem.setModuleStates(0, 0, 0, true);
		recordPose("Target@Odometry", null);
		recordString("drive",
				"end - : " + (interrupted ? "interrupted"
						: "completed") + String.format(" - current: %s, target: %s",
								"" + toString(m_driveSubsystem.getCorrectedPose()),
								toString(m_controllerX.getGoal().position, m_controllerY.getGoal().position,
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
	 * Applies the specified threshold to the specified value.
	 * 
	 * @param value     the value to be thresholded
	 * @param threshold the threshold limit
	 * @return the original value if the absolute value of that value is greater or
	 *         equal to the threshold; the threshold with the original value's sign
	 *         otherwise
	 */
	public static double applyThreshold(double value, double threshold) {
		return Math.abs(value) < threshold ? Math.signum(value) * threshold : value;
	}

	/**
	 * Returns a string representation of the specified {@code Pose2d}.
	 * 
	 * @param pose a {@code Pose2d}
	 * @return a string representation of the specified {@code Pose2d}.
	 */
	public static String toString(Pose2d pose) {
		return toString(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
	}

	/**
	 * Returns a string representation of the specified values.
	 * 
	 * @param x            the x-coordinate value
	 * @param y            the y-coordinate value
	 * @param yawInDegrees the yaw in degrees
	 * @return a string representation of the specified values.
	 */
	public static String toString(double x, double y, double yawInDegrees) {
		return String.format("[%.2f, %.2f, %.1f degrees]", x, y, yawInDegrees);
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordPose(String entryName, Pose2d value) {
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordString(String entryName, String value) {
	}

}