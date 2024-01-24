package hlib.drive;

/**
 * A {@code PoseCalculatorWestCoast} calculates the pose of a west coast drivetrain based on the pose of that drivetrain
 * at an earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorWestCoast extends PoseCalculatorDriveTrain {

	/**
	 * A {@code State} represents the state of a west coast drivetrain at a certain moment.
	 */
	public static class State {

		/**
		 * The left wheel encoder position of a west coast drivetrain.
		 */
		double leftEncoderPosition;

		/**
		 * The right wheel encoder position of a west coast drivetrain.
		 */
		double rightEncoderPosition;

		/**
		 * The yaw value in radians of a west coast drivetrain ({@code null} if the yaw value was not available).
		 */
		Double yaw;

		/**
		 * Constructs a {@code State} representing the state of a west coast drivetrain at a certain moment.
		 * 
		 * @param leftEncoderPosition
		 *            the left wheel encoder position of the west coast drivetrain
		 * @param rightEncoderPosition
		 *            the right wheel encoder position of the west coast drivetrain
		 * @param yawInRadians
		 *            the yaw value in radians of the west coast drivetrain ({@code null} if the yaw value is not
		 *            available)
		 */
		public State(double leftEncoderPosition, double rightEncoderPosition, Double yawInRadians) {
			this.leftEncoderPosition = leftEncoderPosition;
			this.rightEncoderPosition = rightEncoderPosition;
			this.yaw = yawInRadians;
		}

		/**
		 * Returns a string representation of this {@code State}.
		 * 
		 * @return a string representation of this {@code State}
		 */
		@Override
		public String toString() {
			return String.format("left encoder position: %.1f, right encoder position: %.1f, yaw: %.1f degrees",
					leftEncoderPosition, rightEncoderPosition, Math.toDegrees(yaw));
		}
	}

	/**
	 * The most recent {@code State} given to this {@code PoseCalculatorWestCoast}.
	 */
	private State state = null;

	/**
	 * Constructs a {@code PoseCalculatorWestCoast} whose purpose is to calculate the pose of a west coast drivetrain
	 * based on the pose of that drivetrain at an earlier time and the changes in that drivetrain observed via a
	 * gyroscope and wheel encoders.
	 * 
	 * @param width
	 *            the width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters
	 */
	public PoseCalculatorWestCoast(double width) {
		super(width);
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
		return pose(previous, state());
	}

	/**
	 * Calculates the current pose of a west coast drivetrain based on the pose of that drivetrain at an earlier time as
	 * well as the specified and previous wheel encoder and yaw values.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @param state
	 *            a {@code State} representing the current state of a west coast drivetrain
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoast}
	 */
	protected Pose pose(Pose previous, State state) {
		if (this.state == null) {
			this.state = state;
			return previous;
		}
		double changeLeftEncoderPosition = state.leftEncoderPosition - this.state.leftEncoderPosition;
		double changeRightEncoderPosition = state.rightEncoderPosition - this.state.rightEncoderPosition;
		Double changeYaw = state.yaw == null || this.state.yaw == null ? null : state.yaw - this.state.yaw;
		this.state = state;
		return calculate(previous, changeLeftEncoderPosition, changeRightEncoderPosition, changeYaw);
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
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorWestCoast}
	 */
	protected abstract Pose calculate(Pose previous, double changeLeftEncoderPosition,
			double changeRightEncoderPosition, Double changeYaw);

	/**
	 * Returns the {@code Pose} of the center of the left wheels of a west coast drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a west coast drivetrain
	 * @return the {@code Pose} of the center of the left wheels of a west coast drivetrain
	 */
	protected Pose leftPose(Pose pose) {
		return new Pose(new Position(0, width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns the {@code Pose} of the center of the right wheels of a west coast drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a west coast drivetrain
	 * @return the {@code Pose} of the center of the right wheels of a west coast drivetrain
	 */
	protected Pose rightPose(Pose pose) {
		return new Pose(new Position(0, -width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns a {@code State} representing the current state of the west coast drive train.
	 * 
	 * @return a {@code State} representing the current state of the west coast drive train
	 */
	public abstract State state();

}
