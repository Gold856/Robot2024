package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The {@code TurnCommand} rotates the robot by a specific angle in the
 * counter-clockwise direction. It utilizes a {@code ProfiledPIDController} to
 * maintain precision in the rotational movement.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TurnCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code TurnCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier<Double>} that calculates the target angle in degrees by
	 * which the robot should rotate. This {@code Supplier<Double>} is used at
	 * the commencement of this {@code TurnCommand} (i.e., when the scheduler begins
	 * to periodically execute this {@code TurnCommand}).
	 */
	private Supplier<Double> m_targetAngleSupplier;

	/**
	 * The {@code ProfiledPIDController} for controlling the rotational movement.
	 */
	private ProfiledPIDController m_turnController;

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to be used by the
	 *                       {@code TurnCommand}
	 * @param targetAngle
	 *                       the target angle in degrees by which the robot should
	 *                       rotate in the counter-clockwise direction
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TurnCommand(DriveSubsystem driveSubsystem, double targetAngle, double angleTolerance) {
		this(driveSubsystem, () -> targetAngle, angleTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param driveSubsystem      the {@code DriveSubsystem} to be used by the
	 *                            {@code TurnCommand}
	 * @param targetAngleSupplier
	 *                            a {@code Supplier<Double>} that calculates the
	 *                            target angle in degrees by which the robot
	 *                            should rotate in the counter-clockwise direction
	 *                            (used when the scheduler begins
	 *                            to periodically execute this
	 *                            {@code TurnCommand})
	 * @param angleTolerance
	 *                            the angle error in degrees which is tolerable
	 */
	public TurnCommand(DriveSubsystem driveSubsystem, Supplier<Double> targetAngleSupplier, double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_targetAngleSupplier = targetAngleSupplier;
		var constraints = new TrapezoidProfile.Constraints(DriveConstants.kTurnMaxVelocity,
				DriveConstants.kTurnMaxAcceleration);
		m_turnController = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
				DriveConstants.kTurnD,
				constraints);
		m_turnController.setTolerance(angleTolerance);
		m_turnController.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code TurnCommand} (i.e, when the
	 * scheduler begins to periodically execute this {@code TurnCommand}).
	 */
	@Override
	public void initialize() {
		double heading = m_driveSubsystem.getPose().getRotation().getDegrees();
		double goal = heading;
		try {
			goal += m_targetAngleSupplier.get();
		} catch (Exception e) {
		}
		m_turnController.reset(heading);
		m_turnController.setGoal(goal);

		var pose = m_driveSubsystem.getPose();
		SmartDashboard.putString(
				"drive",
				String.format("turn: initialize - heading: %.1f, target heading: %.1f, current pose: %s", heading,
						goal, "" + pose));

	}

	/**
	 * Is invoked periodically by the scheduler until this {@code TurnCommand} is
	 * either ended or interrupted.
	 */
	@Override
	public void execute() {
		double heading = m_driveSubsystem.getPose().getRotation().getDegrees();
		double turnSpeed = m_turnController.calculate(heading);
		turnSpeed = -turnSpeed; // NEGATION if positive turnSpeed: clockwise rotation
		turnSpeed = applyThreshold(turnSpeed, DriveConstants.kMinSpeed); // for convergence
		m_driveSubsystem.setModuleStates(0, 0, turnSpeed, true);
		SmartDashboard.putString(
				"drive", String.format("turn: execute - heading: %.1f, turn speed: %.1f, current pose: %s", heading,
						turnSpeed, "" + m_driveSubsystem.getPose()));
	}

	/**
	 * Is invoked once this {@code TurnCommand} is either ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code TurnCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		double heading = m_driveSubsystem.getPose().getRotation().getDegrees();
		m_driveSubsystem.setModuleStates(0, 0, 0, true);
		SmartDashboard.putString("drive",
				String.format("turn: end - %s : heading: %.1f, target heading: %.1f, current pose: %s",
						(interrupted ? "interrupted" : "completed"), heading,
						m_turnController.getGoal().position, "" + m_driveSubsystem.getPose()));
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

	/**
	 * Applies the specified threshold to the specified value.
	 * 
	 * @param value     the value to be thresholded
	 * @param threshold the threshold limit
	 * @return the original value if the absolute value of that value is greater or
	 *         equal to the threshold; the threshold with the original value's sign
	 *         otherwise
	 */
	public static double applyThreshold(double value, double threshold) {
		return Math.abs(value) < threshold ? Math.signum(value) * threshold : value;
	}

}