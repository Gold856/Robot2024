package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;
	private double m_tolerance;
	private double m_targetDirection;
	private PIDController m_controller = new PIDController(0.1, 0.02, 0);

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	private DriveDistanceCommand(DriveSubsystem subsystem, double amount, double tolerance) {
		m_driveSubsystem = subsystem;
		m_amount = amount;
		m_tolerance = tolerance;
		m_controller.setTolerance(tolerance);
		m_controller.setIZone(0.4);
		addRequirements(subsystem);
	}

	public static SequentialCommandGroup create(DriveSubsystem subsystem, double amount, double tolerance) {
		return new SequentialCommandGroup(
				new SetSteering(subsystem, 0),
				new DriveDistanceCommand(subsystem, amount, tolerance));
	}

	public static SequentialCommandGroup create(DriveSubsystem subsystem, double amount) {
		return new SequentialCommandGroup(
				new SetSteering(subsystem, 0),
				new DriveDistanceCommand(subsystem, amount, 0.1));
	}

	@Override
	public void initialize() {
		double currentPosition = m_driveSubsystem.getModulePositions()[0].distanceMeters;
		m_target = currentPosition + m_amount;

		// With optimize off, encoder distance always increases
		// m_target = currentPosition + Math.abs(m_amount);
		// m_targetDirection = Math.signum(m_amount);

		m_controller.reset();
		m_controller.setSetpoint(m_target);

	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("err", m_controller.getPositionError());
		var out = m_controller.calculate(m_driveSubsystem.getModulePositions()[0].distanceMeters);
		double max = 0.5;
		double min = 0.1;

		if (Math.abs(out) > max) {
			out = Math.signum(out) * max;
		}

		if (Math.abs(out) < min) {
			out = Math.signum(out) * min;
		}

		SmartDashboard.putNumber("out", out);
		// m_driveSubsystem.setModuleStates(m_targetDirection * out, 0, 0, false);
		m_driveSubsystem.setModuleStates(out, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		// Determine whether the target distance has been reached
		// double diff = getDiff();
		// SmartDashboard.putNumber("diff", diff);
		// return diff < m_tolerance;
		return m_controller.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}

	private double getDiff() {
		return Math.abs(m_target - m_driveSubsystem.getModulePositions()[0].distanceMeters);
	}
}
