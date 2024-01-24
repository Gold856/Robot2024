package hlib.drive;

/**
 * A {@code PoseCalculatorSwerveSimple} calculates the pose of a swerve drivetrain based on the pose of that drivetrain
 * at an earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorSwerveSimple extends PoseCalculatorSwerve {

	/**
	 * Constructs a {@code PoseCalculatorSwerveSimple} whose purpose is to calculate the pose of a swerve drivetrain
	 * based on the pose of that drivetrain at an earlier time and the changes in that drivetrain observed via a
	 * gyroscope and wheel encoders.
	 * 
	 * @param width
	 *            the width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters
	 * @param length
	 *            the length (i.e., the distance from the front to the back) of the drivetrain in meters.
	 */
	public PoseCalculatorSwerveSimple(double width, double length) {
		super(width, length);
	}

	/**
	 * Calculates the current pose of a swerve drivetrain based on the pose of that drivetrain at an earlier time as
	 * well as the specified changes in the wheel angles and encoder positions as well as the yaw value.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @param changeWheelAnglesInRadians
	 *            the change in the angles in radians of the front left, front right, back left, and back right wheels
	 *            of a swerve
	 * @param changeWheelEncoderPositions
	 *            the change in the encoder positions in meters of the front left, front right, back left, and back
	 *            right wheels of a swerve drivetrain.
	 * @param changeYaw
	 *            the change in the yaw value of the drivetrain ({@code null} if the yaw values have not been available)
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorSwerve}
	 */
	@Override
	protected Pose calculate(Pose previous, double[] changeWheelAnglesInRadians, double[] changeWheelEncoderPositions,
			Double changeYaw) {
		// TODO
		return null;
	}

}
