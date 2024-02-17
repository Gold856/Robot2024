package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class BangBangDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;
	private double m_tolerance;
	private double m_angle;

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public BangBangDriveCommand(DriveSubsystem subsystem, double amount, double angle, double tolerance) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = tolerance;
		m_angle = angle;
		addRequirements(subsystem);
	}

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public BangBangDriveCommand(DriveSubsystem subsystem, double amount, double angle) {
		this(subsystem, amount, angle, 0.01);
	}

	@Override
	public void initialize() {
		double currentPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		m_target = currentPosition + m_amount;
	}

	@Override
	public void execute() {
		double sign;
		if (m_target > m_driveSubsystem.getModulePositions()[0].distanceMeters) {
			sign = 1;
		} else {
			sign = -1;
		}

		double error = getDiff();
		double kP = 0.1;
		double speed = error * kP;
		if (speed > kMaxSpeed) {
			speed = kMaxSpeed;
		} else if (speed < kMinSpeed) {
			speed = kMinSpeed;
		}

		m_driveSubsystem.setModuleStatesDirect(new SwerveModuleState(speed, Rotation2d.fromDegrees(m_angle)));
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		double diff = getDiff();
		SmartDashboard.putNumber("diff", diff);
		return diff < m_tolerance;
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	private double getDiff() {
		return Math.abs(m_target - m_driveSubsystem.getModulePositions()[0].distanceMeters);
	}
}