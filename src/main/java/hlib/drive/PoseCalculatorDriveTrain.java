package hlib.drive;

/**
 * A {@code PoseCalculatorDriveTrain} calculates the pose of a drivetrain based on the pose of that drivetrain at an
 * earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorDriveTrain implements PoseCalculator {

	/**
	 * The width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters.
	 */
	protected double width;

	/**
	 * Constructs a {@code PoseCalculatorDriveTrain} whose purpose is to calculate the pose of a drivetrain based on the
	 * pose of that drivetrain at an earlier time and the changes in that drivetrain observed via a gyroscope and wheel
	 * encoders.
	 * 
	 * @param width
	 *            the width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters
	 */
	public PoseCalculatorDriveTrain(double width) {
		this.width = width;
	}

	/**
	 * Returns the {@code Pose} of the center of the specified left and right wheels of a swerve drivetrain.
	 * 
	 * @param left
	 *            the {@code Pose} of the left wheels of a swerve drivetrain
	 * @param right
	 *            the {@code Pose} of the right wheels of a swerve drivetrain
	 * @return the {@code Pose} of the center of the specified left and right wheels of a swerve drivetrain
	 */
	protected Pose centerPose(Pose left, Pose right) {
		return new Pose((left.x() + right.x()) / 2, (left.y() + right.y()) / 2,
				left.displacementTo(right).angleInRadians() + Math.PI / 2);
	}

}
