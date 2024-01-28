package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.YDLidarSubsystem;

public class LidarDriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;
	private double m_tolerance;

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public LidarDriveDistanceCommand(DriveSubsystem subsystem, double amount, double tolerance) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = tolerance;
		addRequirements(subsystem);
		addRequirements(YDLidarSubsystem.get());
	}

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance from wall in meters
	 */
	public LidarDriveDistanceCommand(DriveSubsystem subsystem, double amount) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = 0.1;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		double currentPosition = YDLidarSubsystem.get().getDistance(180);
		m_target = currentPosition + m_amount;
	}

	@Override
	public void execute() {
		double sign;
		if (m_target > YDLidarSubsystem.get().getDistance(180)) {
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

		m_driveSubsystem.setModuleStates(speed * sign, 0, 0, false);
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
		return Math.abs(m_target - YDLidarSubsystem.get().getDistance(180));
	}
}