package hlib.drive;

/**
 * A {@code PoseCalculatorWestCoastAdvanced} calculates the pose of a west coast drivetrain based on the pose of that
 * drivetrain at an earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorWestCoastAdvanced extends PoseCalculatorWestCoast {

	/**
	 * Constructs a {@code PoseCalculatorWestCoastAdvanced} whose purpose is to calculate the pose of a west coast
	 * drivetrain based on the pose of that drivetrain at an earlier time and the changes in that drivetrain observed
	 * via a gyroscope and wheel encoders.
	 * 
	 * @param width
	 *            the width of the drivetrain in meters
	 */
	public PoseCalculatorWestCoastAdvanced(double width) {
		super(width);
	}

	/**
	 * Calculates the current pose of a west coast drivetrain based on the pose of that drivetrain at an earlier time as
	 * well as the specified changes in the wheel encoder positions and yaw value.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @param changeLeftEncoderPosition
	 *            the change in the left wheel encoder position of the drivetrain
	 * @param changeRightEncoderPosition
	 *            the change in the right wheel encoder position of the drivetrain
	 * @param changeYaw
	 *            the change in the yaw value of the drivetrain ({@code null} if the yaw values have not been available)
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoastSimple}
	 */
	@Override
	protected Pose calculate(Pose previous, double changeLeftEncoderPosition, double changeRightEncoderPosition,
			Double changeYaw) {
		// TODO
		return null;
	}

}
