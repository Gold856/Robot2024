package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
public class TagAlignCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code ProfiledPIDController} for controlling the rotational movement.
	 */
	private ProfiledPIDController m_turnController;

	private LimeLightSubsystem m_limeLightSubsystem;

	/**
	 * Constructs a {@code TurnCommand}.
	 * 
	 * @param tagID
	 *                       the ID of the target AprilTag
	 * @param angleTolerance
	 *                       the angle error in degrees which is tolerable
	 */
	public TagAlignCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem, double angleTolerance) {
		m_driveSubsystem = driveSubsystem;
		m_limeLightSubsystem = limeLightSubsystem;
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
			goal += m_limeLightSubsystem.getRotationToDetectedTags().getDegrees();
		} catch (Exception e) {
		}
		m_turnController.reset(heading);
		m_turnController.setGoal(goal);
	}

	/**
	 * Is invoked periodically by the scheduler until this {@code TurnCommand} is
	 * either ended or interrupted.
	 */
	@Override
	public void execute() {
		double heading = m_driveSubsystem.getHeading().getDegrees();
		double turnSpeed = m_turnController.calculate(heading);
		turnSpeed = -turnSpeed; // TODO: negation
		// turnSpeed = applyThreshold(turnSpeed, DriveConstants.kMinSpeed);
		m_driveSubsystem.setModuleStates(0, 0, turnSpeed, true);
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