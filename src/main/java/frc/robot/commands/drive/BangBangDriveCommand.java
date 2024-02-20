package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * One position back to another (Bang Bang) drive command uses PIDs
 * 
 * @author Nitya Bajaj
 */
public class BangBangDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target;
	private double m_distance;
	private double m_tolerance;
	private double m_angle;

	/**
	 * Creates a command to drive to a point by specifying a distance and and angle
	 * (robot relative).
	 * 
	 * @param subsystem The drive subsystem.
	 * @param distance  The distance to drive.
	 * @param angle     The angle to drive relative to the robot (CCW+).
	 * @param tolerance The distance tolerance.
	 */
	public BangBangDriveCommand(DriveSubsystem subsystem, double distance, double angle, double tolerance) {
		m_driveSubsystem = subsystem;
		m_distance = distance;
		m_tolerance = tolerance;
		m_angle = angle;
		addRequirements(subsystem);
	}

	/**
	 * Creates a command to drive to a point by specifying a distance and and angle
	 * (robot relative).
	 * 
	 * @param subsystem The drive subsystem.
	 * @param distance  The distance to drive.
	 * @param angle     The angle to drive relative to the robot (CCW+).
	 */
	public BangBangDriveCommand(DriveSubsystem subsystem, double amount, double angle) {
		this(subsystem, amount, angle, 0.01);
	}

	@Override
	public void initialize() {
		m_target = m_driveSubsystem.getModulePositions()[0].distanceMeters + m_distance;
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
		speed = MathUtil.clamp(speed, kMinSpeed, kMaxSpeed);

		m_driveSubsystem.setModuleStatesDirect(new SwerveModuleState(speed * sign, Rotation2d.fromDegrees(m_angle)));
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