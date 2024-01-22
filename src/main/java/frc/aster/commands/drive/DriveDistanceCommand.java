package frc.aster.commands.drive;

import java.util.function.Supplier;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.subsystems.DriveSubsystem;

/**
 * The {@code DriveDistanceCommand} is responsible for moving the robot by a
 * specified distance. It utilizes two
 * {@code ProfiledPIDController}s to precisely control the let and right wheels.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class DriveDistanceCommand extends Command {

	/**
	 * The
	 * {@code Supplier<Double>) that calculates the target distance in meters that the robot should move.
	 * This is used at the commencement of this {@code DriveDistanceCommand} (i.e.,
	 * when the scheduler begins to periodically excute this {@code
	 * DriveDistanceCommand}).
	 */
	private Supplier<Double> m_targetDistanceCalculator;

	/**
	 * The target distance in meters that the robot should move.
	 */
	private double m_targetDistance;

	/**
	 * The {@code ProfiledPIDController} for controlling the left wheels.
	 */
	private ProfiledPIDController m_leftController;

	/**
	 * The {@code ProfiledPIDController} for controlling the right wheels.
	 */
	private ProfiledPIDController m_rightController;

	/**
	 * The left encoder position when this {@code DriveDistanceCommand} is
	 * initialized.
	 */
	private double m_startLeftEncoderPosition;

	/**
	 * The right encoder position when this {@code DriveDistanceCommand} is
	 * initialized.
	 */
	private double m_startRightEncoderPosition;

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
	public DriveDistanceCommand(double targetDistance, double distanceTolerance) {
		this(() -> targetDistance, distanceTolerance);
	}

	/**
	 * Constructs a new {@code DriveDistanceCommand} whose purpose is to move the
	 * robot by the specified distance.
	 * 
	 * @param targetDistanceCalculator
	 *                                 a {@code Supplier<Double>} that calculates
	 *                                 the target distance in meters that the robot
	 *                                 should
	 *                                 move (used when the scheduler begins to
	 *                                 periodically execute this
	 *                                 {@code DriveDistanceCommand})
	 * 
	 * @param distanceTolerance        the distance error in meters which is
	 *                                 tolerable
	 */
	public DriveDistanceCommand(Supplier<Double> targetDistanceCalculator, double distanceTolerance) {
		m_targetDistanceCalculator = targetDistanceCalculator;
		double kP = .2, kI = 0.0, kD = 0.0;
		var constraints = new TrapezoidProfile.Constraints(3, 2);
		m_leftController = new ProfiledPIDController(kP, kI, kD, constraints);
		m_rightController = new ProfiledPIDController(kP, kI, kD, constraints);
		m_leftController.setTolerance(distanceTolerance);
		m_rightController.setTolerance(distanceTolerance);
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is invoked at the commencement of this {@code DriveDistanceCommand} (i.e,
	 * when the
	 * scheduler begins to periodically execute this {@code DriveDistanceCommand}).
	 */
	@Override
	public void initialize() {
		m_startLeftEncoderPosition = DriveSubsystem.get().getLeftEncoderPosition();
		m_startRightEncoderPosition = DriveSubsystem.get().getRightEncoderPosition();
		try {
			m_targetDistance = m_targetDistanceCalculator.get();
		} catch (Exception e) {
			m_targetDistance = 0;
		}
		SmartDashboard.putString(
				"drive",
				String.format(
						"distance: initialize - left encoder position: %.1f, right encoder position: %.1f, target distance: %.1f",
						m_startLeftEncoderPosition,
						m_startRightEncoderPosition, m_targetDistance));
	}

	/**
	 * Is invoked periodically by the scheduler while it is in charge of executing
	 * this
	 * {@code DriveDistanceCommand}.
	 */
	@Override
	public void execute() {
		var leftEncoderPosition = DriveSubsystem.get().getLeftEncoderPosition();
		var rightEncoderPosition = DriveSubsystem.get().getRightEncoderPosition();
		double leftSpeed = m_leftController.calculate(leftEncoderPosition - m_startLeftEncoderPosition,
				m_targetDistance);
		double rightSpeed = m_rightController.calculate(rightEncoderPosition - m_startRightEncoderPosition,
				m_targetDistance);
		DriveSubsystem.get().tankDrive(leftSpeed, rightSpeed);
		SmartDashboard.putString(
				"drive",
				String.format(
						"distance: execute - left encoder position: %.1f, right encoder position: %.1f, left speed: %.1f, right speed: %.1f",
						leftEncoderPosition, rightEncoderPosition,
						leftSpeed, rightSpeed));
	}

	/**
	 * Is invoked once this {@code DriveDistanceCommand} is ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code DriveDistanceCommand} was
	 *                    interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
		SmartDashboard.putString("drive", "distance: end - " + interrupted);

	}

	/**
	 * Determines whether or not this {@code DriveDistanceCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveDistanceCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return Math.abs(
				DriveSubsystem.get().getLeftEncoderPosition() - m_startLeftEncoderPosition - m_targetDistance) < .1
				&& Math.abs(DriveSubsystem.get().getRightEncoderPosition() - m_startRightEncoderPosition
						- m_targetDistance) < .1;// m_leftController.atSetpoint()
		// &&
		// m_rightController.atSetpoint();
	}
}