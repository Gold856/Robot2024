package frc.aster.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants.DriveConstants;
import frc.aster.subsystems.DriveSubsystem;

/**
 * The {@code TurnCommand} rotates the robot by a specific angle in the
 * counter-clockwise direction. It utilizes a {@code PIDController} to maintain
 * precision in the rotational movement.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TurnCommand extends Command {

	/**
	 * The {@code Supplier<Double>} that calculates the target angle in degrees by
	 * which the robot should rotate. This {@code Supplier<Double>} is used at
	 * the commencement of this {@code TurnCommand} (i.e., when the scheduler begins
	 * to periodically execute this {@code TurnCommand}).
	 */
	private Supplier<Double> m_targetAngleCalculator;

	/**
	 * The {@code PIDController} for controlling the rotational movement.
	 */
	private ProfiledPIDController m_turnController;

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param targetAngle
	 *                       the target angle in degrees by which the robot should
	 *                       rotate in the counter-clockwise direction
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TurnCommand(double targetAngle, double angleTolerance) {
		this(() -> targetAngle, angleTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param targetAngleCalculator
	 *                              a {@code Supplier<Double>} that calculates the
	 *                              target angle in degrees by which the robot
	 *                              should rotate in the counter-clockwise direction
	 *                              (used when the scheduler begins
	 *                              to periodically execute this
	 *                              {@code TurnCommand})
	 * @param angleTolerance
	 *                              the angle error in degrees which is tolerable
	 */
	public TurnCommand(Supplier<Double> targetAngleCalculator, double angleTolerance) {
		m_targetAngleCalculator = targetAngleCalculator;
		var constraints = new TrapezoidProfile.Constraints(120, 120);
		m_turnController = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
				DriveConstants.kTurnD,
				constraints);
		m_turnController.setTolerance(angleTolerance);
		m_turnController.enableContinuousInput(-180, 180);
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Is invoked at the commencement of this {@code TurnCommand} (i.e, when the
	 * scheduler begins to periodically execute this {@code TurnCommand}).
	 */
	@Override
	public void initialize() {
		double heading = DriveSubsystem.get().getHeading();
		double goal = heading;
		try {
			goal += m_targetAngleCalculator.get();
		} catch (Exception e) {
		}
		m_turnController.reset(heading);
		m_turnController.setGoal(goal);
		SmartDashboard.putString("drive",
				String.format("turn: initialize - heading: %.1f, target heading: %.1f", heading,
						goal));
	}

	/**
	 * Is invoked periodically by the scheduler while it is in charge of executing
	 * this {@code TurnCommand}.
	 */
	@Override
	public void execute() {
		double heading = DriveSubsystem.get().getHeading();
		double turnSpeed = m_turnController.calculate(heading);
		turnSpeed = Math.abs(turnSpeed) < DriveConstants.kTurnMinSpeed
				? Math.signum(turnSpeed) * DriveConstants.kTurnMinSpeed
				: turnSpeed;
		DriveSubsystem.get().tankDrive(-turnSpeed, turnSpeed);
		SmartDashboard.putString("drive",
				String.format("turn: execute - heading: %.1f, turn speed: %.1f", heading, turnSpeed));
	}

	/**
	 * Is invoked once this {@code TurnCommand} is ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code TurnCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		DriveSubsystem.get().tankDrive(0, 0);
		SmartDashboard.putString("drive",
				String.format("turn: end - %s : heading: %.1f, target heading: %.1f",
						(interrupted ? "interrupted" : "completed"), DriveSubsystem.get().getHeading(),
						m_turnController.getGoal().position));
	}

	/**
	 * Determines whether or not this {@code TurnCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code TurnCommand} needs to end; {@code false}
	 *         otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_turnController.atGoal();
	}
}