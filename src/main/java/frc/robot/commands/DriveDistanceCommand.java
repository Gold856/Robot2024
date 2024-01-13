package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public DriveDistanceCommand(DriveSubsystem subsystem, double amount) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		addRequirements(subsystem);
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
		double speed;
		if (getDiff() > 2) {
			speed = 0.2;
		} else if (getDiff() > 1) {
			speed = 0.1;
		} else {
			speed = getDiff() * 0.1;
		}
		var moduleStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(sign * speed, 0, 0), false);
		m_driveSubsystem.setSwerveStates(moduleStates);
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		double diff = getDiff();
		SmartDashboard.putNumber("diff", diff);
		return diff < 0.2;
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	private double getDiff() {
		return Math.abs(m_target - m_driveSubsystem.getModulePositions()[0].distanceMeters);
	}
}
