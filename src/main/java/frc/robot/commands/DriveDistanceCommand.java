package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private double m_target; // if distance, in meters; if angle, in degrees
	private double m_amount;
	private double m_tolerance;
	private double m_targetDirection;
	private ProfiledPIDController m_controller = new ProfiledPIDController(0.1, 0.02, 0,
			new Constraints(3, 2));

	/***
	 * Autonomous command to drive straight
	 * 
	 * @param amount
	 *               amount is distance in meters
	 */
	public DriveDistanceCommand(DriveSubsystem subsystem, double amount, double tolerance) {
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
		double currentPosition = m_driveSubsystem.getPose().getX();
		m_target = currentPosition + m_amount;

		// With optimize off, encoder distance always increases
		// m_target = currentPosition + Math.abs(m_amount);
		// m_targetDirection = Math.signum(m_amount);

		m_controller.reset(0);
		m_controller.setGoal(m_target);

	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("err", m_controller.getPositionError());
		var out = m_controller.calculate(m_driveSubsystem.getPose().getX());
		SmartDashboard.putNumber("out", out);
		SmartDashboard.putNumber("Setpoint Position", m_controller.getSetpoint().position);
		SmartDashboard.putNumber("Setpoint Velocity", m_controller.getSetpoint().velocity);
		// m_driveSubsystem.setModuleStates(m_targetDirection * out, 0, 0, false);
		m_driveSubsystem.setModuleStates(out, 0, 0, true);
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
