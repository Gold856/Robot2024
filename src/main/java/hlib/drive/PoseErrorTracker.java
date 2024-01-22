package hlib.drive;

/**
 * A {@code PoseErrorTracker} can keep track of discrepancies in {@code Pose}s.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PoseErrorTracker {

	/**
	 * The {@code Pose} containing the largest discrepancy in each dimension.
	 */
	protected Pose largestPoseError = new Pose(0, 0, 0);

	/**
	 * Constructs a {@code PoseErrorTracker}.
	 */
	public PoseErrorTracker() {
	}

	/**
	 * Calculates the discrepancies between the specified {@code Pose}s.
	 * 
	 * @param pose
	 *            a {@code Pose}
	 * @param reference
	 *            a reference {@code Pose} for comparison
	 * @return a {@code Pose} representing the difference between the {@code Pose}s ({@code null} if either of the given
	 *         {@code Pose}s is {@code null})
	 */
	public static Pose error(Pose pose, Pose reference) {
		if (pose == null || reference == null)
			return null;
		return new Pose(pose.x() - reference.x(), pose.y() - reference.y(), pose.yaw - reference.yaw);
	}

	/**
	 * Updates this {@code PoseErrorTracker} based on the difference between the specified {@code Pose}s.
	 * 
	 * @param pose
	 *            a {@code Pose}
	 * @param reference
	 *            a reference {@code Pose} for comparison
	 */
	public void update(Pose pose, Pose reference) {
		update(error(pose, reference));
	}

	/**
	 * Returns the {@code Pose} containing the largest discrepancy in each dimension.
	 * 
	 * @return the {@code Pose} containing the largest discrepancy in each dimension
	 */
	public Pose largestPoseError() {
		return largestPoseError;
	}

	/**
	 * Updates this {@code PoseErrorTracker} based on the error values contained in the specified {@code Pose}.
	 * 
	 * @param error
	 *            a {@code Pose} containing error values
	 */
	protected void update(Pose error) {
		if (error != null)
			this.largestPoseError = new Pose(maxError(error.x(), largestPoseError.x()),
					maxError(error.y(), largestPoseError.y()), maxError(error.yaw, largestPoseError.yaw));
	}

	/**
	 * Returns the larger of the specified error values.
	 * 
	 * @param e1
	 *            an error value
	 * @param e2
	 *            an error value
	 * @return the larger of the specified error values
	 */
	protected double maxError(double e1, double e2) {
		if (Math.abs(e1) > Math.abs(e2))
			return e1;
		else
			return e2;
	}

}
