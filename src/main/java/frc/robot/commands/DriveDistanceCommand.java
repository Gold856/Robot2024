package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_distance;
	private final ProfiledPIDController m_controller = new ProfiledPIDController(0.45, 0.05, kD,
			new Constraints(16, 1));

	/**
	 * Constructs a DriveDistanceCommand.
	 * 
	 * @param subsystem The subsystem
	 * @param distance  The distance in meters
	 * @param tolerance The tolerance in meters
	 */
	public DriveDistanceCommand(DriveSubsystem subsystem, double distance, double tolerance) {
		m_driveSubsystem = subsystem;
		m_distance = distance;
		m_controller.setTolerance(tolerance);
		m_controller.setIZone(0.4);
		addRequirements(subsystem);
	}

	/**
	 * Constructs a DriveDistanceCommand.
	 * 
	 * @param subsystem The subsystem
	 * @param distance  The distance in meters
	 */
	public DriveDistanceCommand(DriveSubsystem subsystem, double distance) {
		this(subsystem, distance, 0.1);
	}

	@Override
	public void initialize() {
		m_controller.setGoal(m_driveSubsystem.getModulePositions()[0].distanceMeters + m_distance);
	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("err", m_controller.getPositionError());
		var out = m_controller.calculate(m_driveSubsystem.getModulePositions()[0].distanceMeters);
		SmartDashboard.putNumber("out", out);
		m_driveSubsystem.setModuleStates(out + 0., 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return m_controller.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.stopDriving();
	}
}
