package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.SwerveModule;

public class DriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	double m_distance = 0; // if distance, in meters; if angle, in degrees

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public DriveDistanceCommand(double amount) {
		m_driveSubsystem = DriveSubsystem.get();
		m_distance = amount;
		addRequirements(DriveSubsystem.get());
	}

	@Override
	public void initialize() {
		double currentPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		m_distance += currentPosition;
	}

	@Override
	public void execute() {
		m_driveSubsystem.setSwerveStates((m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(.1, 0, 0), false)));
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		double currentPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		return ((m_distance - currentPosition) < 0.1);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}
