package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The {@code TagAlignCommand} rotates the robot to the mid point of all the
 * AprilTags detected by a {@code LimeLightSubsystem}. It utilizes a
 * {@code ProfiledPIDController} to
 * maintain precision in the rotational movement.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class TagDistanceAlignCommand extends DriveCommand {

	/**
	 * Constructs a {@code TagAlignCommand}.
	 * 
	 * @param driveSubsystem    the {@code DriveSubsystem} used by the
	 *                          {@code TagAlignCommand}
	 * @param distanceToTag
	 *                          the target distance between the robot and the in
	 *                          meters which is tolerable
	 * @param distanceTolerance
	 *                          the distance error in meters which is tolerable
	 * @param angleTolerance
	 *                          the angle error in degrees which is tolerable
	 */
	public TagDistanceAlignCommand(DriveSubsystem driveSubsystem, double distanceToTag, double distanceTolerance,
			double angleTolerance) {
		super(driveSubsystem, () -> {
			var transform = TagAlignCommand.transformationToTagPosition();
			var translation = transform.getTranslation();
			double norm = translation.getNorm();
			if (norm == 0)
				return driveSubsystem.getPose();
			translation = translation.times(1 - distanceToTag / norm);
			return driveSubsystem.getPose().plus(new Transform2d(translation, transform.getRotation()));
		}, distanceTolerance, angleTolerance);
	}

}