package hlib.drive;

/**
 * A {@code PoseCalculatorWestCoast} calculates the pose of a swerve drivetrain based on the pose of that drivetrain at
 * an earlier time and the changes in that drivetrain observed via a gyroscope and wheel encoders.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public abstract class PoseCalculatorSwerve implements PoseCalculator {

	/**
	 * A {@code State} represents the state of a swerve drivetrain at a certain moment.
	 */
	public static class State {

		/**
		 * The angles in radians of the front left, front right, back left, and back right wheels of a swerve
		 * drivetrain.
		 */
		double[] wheelAngles;

		/**
		 * The encoder positions in meters of the front left, front right, back left, and back right wheels of a swerve
		 * drivetrain.
		 */
		double[] wheelEncoderPositions;

		/**
		 * The yaw value in radians of a swerve drivetrain ({@code null} if the yaw value was not available).
		 */
		Double yaw;

		/**
		 * Constructs a {@code State} representing the state of a swerve drivetrain at a certain moment.
		 * 
		 * @param wheelAnglesInRadians
		 *            the angles in radians of the front left, front right, back left, and back right wheels of a swerve
		 *            drivetrain
		 * @param wheelEncoderPositions
		 *            the encoder positions in meters of the front left, front right, back left, and back right wheels
		 *            of a swerve drivetrain
		 * @param yawInRadians
		 *            the yaw value in radians of the swerve drivetrain ({@code null} if the yaw value is not available)
		 */
		public State(double[] wheelAnglesInRadians, double[] wheelEncoderPositions, Double yawInRadians) {
			this.wheelAngles = wheelAnglesInRadians;
			this.wheelEncoderPositions = wheelEncoderPositions;
			this.yaw = yawInRadians;
		}

		/**
		 * Returns a string representation of this {@code State}.
		 * 
		 * @return a string representation of this {@code State}
		 */
		@Override
		public String toString() {
			return String.format(
					"wheel angles in degrees: [%.1f, %.1f, %.1f, %.1f], wheen encoder positions: [%.1f, %.1f, %.1f, %.1f], yaw: %.1f degrees",
					wheelAngles[0], wheelAngles[1], wheelAngles[2], wheelAngles[3], wheelEncoderPositions[0],
					wheelEncoderPositions[1], wheelEncoderPositions[2], wheelEncoderPositions[3], yaw * 180 / Math.PI);
		}
	}

	/**
	 * The width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters.
	 */
	private double width;

	/**
	 * The length (i.e., the distance from the front to the back) of the drivetrain in meters.
	 */
	private double length;

	/**
	 * The most recent {@code State} given to this {@code PoseCalculatorSwerve}.
	 */
	private State state = null;

	/**
	 * Constructs a {@code PoseCalculatorSwerve} whose purpose is to calculate the pose of a swerve drivetrain based
	 * on the pose of that drivetrain at an earlier time and the changes in that drivetrain observed via a gyroscope and
	 * wheel encoders.
	 * 
	 * @param width
	 *            the width (i.e., the distance from one wheel to the opposite wheel) of the drivetrain in meters
	 * @param length
	 *            the length (i.e., the distance from the front to the back) of the drivetrain in meters.
	 */
	public PoseCalculatorSwerve(double width, double length) {
		this.width = width;
		this.length = length;
	}

	/**
	 * Calculates the current pose of a swerve drivetrain based on the pose of that drivetrain at an earlier time and
	 * the changes in that drivetrain observed via a gyroscope and wheel encoders.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorSwerve}
	 */
	@Override
	public Pose pose(Pose previous) {
		return pose(previous, state());
	}

	/**
	 * Calculates the current pose of a swerve drivetrain based on the pose of that drivetrain at an earlier time as
	 * well as the specified and previous wheel encoder and yaw values.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the drivetrain at an earlier time
	 * @param state
	 *            a {@code State} representing the current state of a swerve drivetrain
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculatorSwerve}
	 */
	protected Pose pose(Pose previous, State state) {
		if (this.state == null) {
			this.state = state;
			return previous;
		}
		double[] changeWheelAnglesInRadians = difference(state.wheelAngles, this.state.wheelAngles);
		double[] changeWheelEncoderPositions = difference(state.wheelEncoderPositions,
				this.state.wheelEncoderPositions);
		Double changeYaw = state.yaw == null || this.state.yaw == null ? null : state.yaw - this.state.yaw;
		this.state = state;
		return calculate(previous, changeWheelAnglesInRadians, changeWheelEncoderPositions, changeYaw);
	}

	/**
	 * Calculates the difference from the first {@code double} array to the second {@code double} array.
	 * 
	 * @param a1
	 *            the first {@code double} array
	 * @param a2
	 *            the second {@code double} array
	 * @return a {@code double} array, which is the difference from the first {@code double} array to the second
	 *         {@code double} array
	 */
	private double[] difference(double[] a1, double[] a2) {
		double[] difference = new double[4];
		for (int i = 0; i < difference.length; i++)
			difference[i] = a1[i] - a2[i];
		return difference;
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
	protected abstract Pose calculate(Pose previous, double[] changeWheelAnglesInRadians, double[] changeWheelEncoderPositions,
			Double changeYaw);

	/**
	 * Returns the {@code Pose} of the center of the front left wheels of a swerve drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a swerve drivetrain
	 * @return the {@code Pose} of the center of the front left wheels of a swerve drivetrain
	 */
	protected Pose frontLeftPose(Pose pose) {
		return new Pose(new Position(length / 2, width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns the {@code Pose} of the center of the front right wheels of a swerve drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a swerve drivetrain
	 * @return the {@code Pose} of the center of the front right wheels of a swerve drivetrain
	 */
	protected Pose frontRightPose(Pose pose) {
		return new Pose(new Position(length / 2, -width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns the {@code Pose} of the center of the front left wheels of a swerve drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a swerve drivetrain
	 * @return the {@code Pose} of the center of the front left wheels of a swerve drivetrain
	 */
	protected Pose backLeftPose(Pose pose) {
		return new Pose(new Position(-length / 2, width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns the {@code Pose} of the center of the back right wheels of a swerve drivetrain.
	 * 
	 * @param pose
	 *            the {@code Pose} of the center of a swerve drivetrain
	 * @return the {@code Pose} of the center of the back right wheels of a swerve drivetrain
	 */
	protected Pose backRightPose(Pose pose) {
		return new Pose(new Position(-length / 2, -width / 2).rotate(pose.yaw).translate(pose), pose.yaw);
	}

	/**
	 * Returns a {@code State} representing the current state of the swerve drive train.
	 * 
	 * @return a {@code State} representing the current state of the swerve drive train
	 */
	public abstract State state();

}
