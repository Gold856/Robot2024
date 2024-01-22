package hlib.drive;

/**
 * A {@code PoseEstimatorInconsistencyTolerant} estimates the pose of an object. It resets itself if it consecutively
 * rejects a certain number of samples {@code Pose}s as outliers.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimatorInconsistencyTolerant extends PoseEstimator {

	/**
	 * The number of consecutive rejections of sample {@code Pose}s needed to reset this
	 * {@code PoseEstimatorInconsistencyTolerant}.
	 */
	int rejectionLimit;

	/**
	 * The number of consecutive sample {@code Pose}s that have been rejected as outliers.
	 */
	int rejections = 0;

	/**
	 * Constructs a {@code PoseEstimatorInconsistencyTolerant}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (a sample {@code Pose} is considered an outlier and
	 *            rejected if its distance from the estimated {@code Pose} is larger than this threshold)
	 * @param rejectionLimit
	 *            the number of rejections needed to reset the {@code PoseEstimatorInconsistencyTolerant}
	 */

	public PoseEstimatorInconsistencyTolerant(double distanceThreshold, int rejectionLimit) {
		super(distanceThreshold);
		this.rejectionLimit = rejectionLimit;
	}

	/**
	 * Determines whether or not the specified sample {@code Pose} is an outlier.
	 * 
	 * @param sample
	 *            a sample {@code Pose}
	 * @return {@code true} if either the x- or y-coordinate value of the sample {@code Pose} is different by more than
	 *         the threshold compared to the estimated {@code Pose} maintained by the
	 *         {@code PoseEstimatorInconsistencyTolerant}; {@code false} otherwise
	 */
	protected boolean isOutlier(Pose sample) {
		if (super.isOutlier(sample)) {
			if (++rejections > rejectionLimit)
				reset();
			return true;
		}
		if (sample != null && this.estimatedPose != null)
			rejections = 0;
		return false;
	}

	/**
	 * Resets this {@code PoseEstimatorInconsistencyTolerant}.
	 */
	protected void reset() {
		System.out.println("resetting the pose estimator...");
		estimatedPose = null;
		rejections = 0;
	}

}
