package hlib.drive;

/**
 * A {@code PoseCalculatorWestCoast} calculates the pose of a west coast drivetrain based on the pose of that drivetrain
 * at an earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorWestCoast implements PoseCalculator {

	/**
	 * The last left wheel encoder position provided to this {@code PoseCalculatorWestCoast}.
	 */
	protected Double leftEncoderPosition = null;

	/**
	 * The last right wheel encoder position provided to this {@code PoseCalculatorWestCoast}.
	 */
	protected Double rightEncoderPosition = null;

	/**
	 * The last yaw value in radians provided to this {@code PoseCalculatorWestCoast}.
	 */
	protected Double yaw = null;

	/**
	 * The width of the drivetrain in meters.
	 */
	protected double width;

	/**
	 * Constructs a {@code PoseCalculatorWestCoast} whose purpose is to calculate the pose of a west coast drivetrain
	 * based on the pose of that drivetrain at an earlier time and the changes in that drivetrain observed via a
	 * gyroscope and wheel encoders.
	 * 
	 * @param width
	 *            the width of the drivetrain in meters
	 */
	public PoseCalculatorWestCoast(double width) {
		this.width = width;
	}

	/**
	 * Calculates the current pose of a west coast drivetrain based on the pose of that drivetrain at an earlier time
	 * and the changes in that drivetrain observed via a gyroscope and wheel encoders.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoast}
	 */
	@Override
	public Pose pose(Pose previous) {
		return pose(previous, leftEncoderPosition(), rightEncoderPosition(), yawInRadians());
	}

	/**
	 * Calculates the current pose of a west coast drivetrain based on the pose of that drivetrain at an earlier time as
	 * well as the specified and previous wheel encoder and yaw values.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @param leftEncoderPosition
	 *            the current left wheel encoder position of the drivetrain
	 * @param rightEncoderPosition
	 *            the current right wheel encoder position of the drivetrain
	 * @param yaw
	 *            the current yaw value in radians of the drivetrain
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoast}
	 */
	protected Pose pose(Pose previous, double leftEncoderPosition, double rightEncoderPosition, double yaw) {
		if (this.leftEncoderPosition == null) {
			this.leftEncoderPosition = leftEncoderPosition;
			this.rightEncoderPosition = rightEncoderPosition;
			this.yaw = yaw;
			return null;
		}
		double leftDisplacement = leftEncoderPosition - this.leftEncoderPosition;
		double rightDisplacement = rightEncoderPosition - this.rightEncoderPosition;
		double augularDisplacement = yaw - this.yaw;
		this.leftEncoderPosition = leftEncoderPosition;
		this.rightEncoderPosition = rightEncoderPosition;
		this.yaw = yaw;
		return calculate(previous, leftDisplacement, rightDisplacement, augularDisplacement);
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
	 *            the change in the yaw value of the drivetrain
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoast}
	 */
	protected Pose calculate(Pose previous, double changeLeftEncoderPosition, double changeRightEncoderPosition,
			double changeYaw) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * Returns the current position of the left wheel motors in terms of meters (positive: forward).
	 * 
	 * @return the current position of the left wheel motors in terms of meters (positive: forward)
	 */
	public abstract double leftEncoderPosition();

	/**
	 * Returns the current position of the right wheel motors in terms of meters (positive: forward)
	 * 
	 * @return the current position of the right wheel motors in terms of meters (positive: forward)
	 */
	public abstract double rightEncoderPosition();

	/**
	 * Returns the current yaw value in radians of the west coast drivetrain.
	 * 
	 * @return the current yaw value in radians of the west coast drivetrain
	 */
	public abstract double yawInRadians();

}
