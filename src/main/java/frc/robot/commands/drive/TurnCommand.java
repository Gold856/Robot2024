package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

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
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
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
	 * @param targetPosition
	 *                       the target position
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TurnCommand(DriveSubsystem driveSubsystem, Translation2d targetPosition,
			LimeLightSubsystem limeLieghLightSubsystem, double angleTolerance) {
		this(driveSubsystem, () -> limeLieghLightSubsystem.getRotation(targetPosition).getDegrees(), angleTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param tagID
	 *                       the ID of the target AprilTag
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TurnCommand(DriveSubsystem driveSubsystem, String tagID,
			LimeLightSubsystem limeLieghLightSubsystem, double angleTolerance) {
		this(driveSubsystem, () -> limeLieghLightSubsystem.getRotation(tagID).getDegrees(), angleTolerance);
	}

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
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
		double heading = m_driveSubsystem.getHeading().getDegrees();
		double goal = heading;
		try {
			goal += m_targetAngleSupplier.get();
		} catch (Exception e) {
		}
		m_turnController.reset(heading);
		m_turnController.setGoal(goal);

		var pose = m_driveSubsystem.getPose();
		recordPose("BotPose@Odometry", pose);
		recordString(
				"drive",
				String.format("turn: initialize - heading: %.1f, target heading: %.1f, current pose: %s", heading,
						goal, toString(pose)));

	}

	/**
	 * Is invoked periodically by the scheduler until this {@code TurnCommand} is
	 * either ended or interrupted.
	 */
	@Override
	public void execute() {
		double heading = m_driveSubsystem.getHeading().getDegrees();
		double turnSpeed = m_turnController.calculate(heading);
		// turnSpeed = applyThreshold(turnSpeed, DriveConstants.kMinSpeed);
		m_driveSubsystem.setModuleStates(0, 0, turnSpeed, true);

		var pose = m_driveSubsystem.getPose();
		recordPose("BotPose@Odometry", pose);
		recordString(
				"drive", String.format("turn: execute - heading: %.1f, turn speed: %.1f, current pose: %s", heading,
						turnSpeed, toString(m_driveSubsystem.getPose())));
	}

	/**
	 * Is invoked once this {@code TurnCommand} is either ended or interrupted.
	 * 
	 * @param interrupted
	 *                    indicates if this {@code TurnCommand} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.setModuleStates(0, 0, 0, true);

		recordString("drive",
				String.format("turn: end - %s : heading: %.1f, target heading: %.1f, current pose: %s",
						(interrupted ? "interrupted" : "completed"), m_driveSubsystem.getHeading().getDegrees(),
						m_turnController.getGoal().position, toString(m_driveSubsystem.getPose())));
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

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordPose(String entryName, Pose2d value) {
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordString(String entryName, String value) {
	}

	/**
	 * Returns a string representation of the specified {@code Pose2d}.
	 * 
	 * @param pose a {@code Pose2d}
	 * @return a string representation of the specified {@code Pose2d}.
	 */
	public static String toString(Pose2d pose) {
		return toString(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
	}

	/**
	 * Returns a string representation of the specified values.
	 * 
	 * @param x            the x-coordinate value
	 * @param y            the y-coordinate value
	 * @param yawInDegrees the yaw in degrees
	 * @return a string representation of the specified values.
	 */
	public static String toString(double x, double y, double yawInDegrees) {
		return String.format("[%.2f, %.2f, %.1f degrees]", x, y, yawInDegrees);
	}

}