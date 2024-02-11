package frc.robot.commands.drive;

import java.util.function.Supplier;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/**
 * The {@code DriveDistanceCommand} is responsible for moving the robot by a
 * specified distance. It utilizes two {@code ProfiledPIDController}s to
 * precisely control the left and right wheels.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class DriveDistanceCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveDistanceCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier<Double>} that calculates the target distance in meters
	 * that the robot should move.
	 * This is used at the commencement of this {@code DriveDistanceCommand} (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveDistanceCommand}).
	 */
	private Supplier<Double> m_targetDistanceSupplier;

	/**
	 * The {@code ProfiledPIDController} for controlling the left wheels.
	 */
	private ProfiledPIDController m_controller;

	/**
	 * Constructs a new {@code DriveDistanceCommand} whose purpose is to move the
	 * robot by the specified distance.
	 * 
	 * @param targetDistance
	 *                          the target distance in meters that the robot should
	 *                          move
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 */
	public DriveDistanceCommand(DriveSubsystem driveSubsystem, double targetDistance, double distanceTolerance) {
		this(driveSubsystem, () -> targetDistance, distanceTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param targetPosition
	 *                       the target position
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public DriveDistanceCommand(DriveSubsystem driveSubsystem, Translation2d targetPosition, double distanceToTarget,
			LimeLightSubsystem limeLieghLightSubsystem, double distanceTolerance) {
		this(driveSubsystem, () -> limeLieghLightSubsystem.getDistance(targetPosition) - distanceToTarget,
				distanceTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param tagID
	 *                       the ID of the target AprilTag
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public DriveDistanceCommand(DriveSubsystem driveSubsystem, String tagID, double distanceToTarget,
			LimeLightSubsystem limeLieghLightSubsystem, double distanceTolerance) {
		this(driveSubsystem, () -> limeLieghLightSubsystem.getDistance(tagID) - distanceToTarget, distanceTolerance);
	}

	/**
	 * Constructs a new {@code DriveDistanceCommand} whose purpose is to move the
	 * robot by the specified distance.
	 * 
	 * @param targetDistanceSupplier
	 *                               a {@code Supplier<Double>} that calculates
	 *                               the target distance in meters that the robot
	 *                               should
	 *                               move (used when the scheduler begins to
	 *                               periodically execute this
	 *                               {@code DriveDistanceCommand})
	 * 
	 * @param distanceTolerance      the distance error in meters which is
	 *                               tolerable
	 */
	public DriveDistanceCommand(DriveSubsystem driveSubsystem, Supplier<Double> targetDistanceSupplier,
			double distanceTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_targetDistanceSupplier = targetDistanceSupplier;
		var constraints = new TrapezoidProfile.Constraints(Constants.DriveConstants.kDriveMaxVelocity,
				Constants.DriveConstants.kDriveMaxAcceleration);
		m_controller = new ProfiledPIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI,
				Constants.DriveConstants.kDriveD, constraints);
		m_controller.setTolerance(distanceTolerance);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveDistanceCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveDistanceCommand}).
	 */
	@Override
	public void initialize() {
		var starEncoderPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		var targetEncoderPosition = starEncoderPosition;
		try {
			targetEncoderPosition += m_targetDistanceSupplier.get();
		} catch (Exception e) {
		}
		m_controller.reset(starEncoderPosition);
		m_controller.setGoal(targetEncoderPosition);

		var pose = m_driveSubsystem.getPose();
		recordPose("BotPose@Odometry", pose);
		recordString("drive",
				String.format(
						"distance: initialize - encoder position: %.1f, target encoder position: %.1f, current pose: %s",
						starEncoderPosition,
						targetEncoderPosition, TurnCommand.toString(pose)));
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveDistanceCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		var encoderPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		double speed = m_controller.calculate(encoderPosition);
		// speed = TurnCommand.applyThreshold(speed, DriveConstants.kMinSpeed);
		m_driveSubsystem.setModuleStates(speed, 0, 0, false);

		var pose = m_driveSubsystem.getPose();
		recordPose("BotPose@Odometry", pose);
		recordString("drive",
				String.format(
						"distance: execute - encoder position: %.1f, speed: %.1f, current pose: %s",
						encoderPosition,
						speed, TurnCommand.toString(m_driveSubsystem.getPose())));
	}

	/**
	 * Is invoked once this {@code DriveDistanceCommand} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code DriveDistanceCommand} was
	 *                    interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.setModuleStates(0, 0, 0, true);

		recordString("drive",
				String.format(
						"distance: end - %s, encoder position: %.1f, target encoder position: %.1f, current pose: %s",
						(interrupted ? "interrupted" : "completed"),
						m_driveSubsystem.getModulePositions()[0].distanceMeters, m_controller.getGoal().position,
						TurnCommand.toString(m_driveSubsystem.getPose())));
	}

	/**
	 * Determines whether or not this {@code DriveDistanceCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveDistanceCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controller.atGoal();
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
