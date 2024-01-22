package hlib.drive;

/**
 * A {@code PoseEstimatorWeighted} estimates the pose of an object by applying a certain weight to each sample
 * {@code Pose}.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimatorWeighted extends PoseEstimatorInconsistencyTolerant {

	/**
	 * The weight applied to each sample {@code Pose}.
	 */
	protected double weight;

	/**
	 * Constructs a {@code PoseEstimatorWeighted}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (a sample {@code Pose} is considered an outlier and
	 *            rejected if its distance from the estimated {@code Pose} is larger than this threshold)
	 * @param rejectionLimit
	 *            the number of rejections needed to reset the {@code PoseEstimatorWeighted}
	 * @param weight
	 *            the weight of each sample {@code Pose}
	 */
	public PoseEstimatorWeighted(double distanceThreshold, int rejectionLimit, double weight) {
		super(distanceThreshold, rejectionLimit);
		this.weight = weight;
	}

	/**
	 * Updates the estimated {@code Pose} based on the specified sample {@code Pose}.
	 * 
	 * @param sample
	 *            a sample {@code Pose}
	 */
	@Override
	public void estimatedPose(Pose sample) {
		if (sample != null) {
			errorTracker.update(sample, this.estimatedPose);
			this.estimatedPose = weightedSum(sample, weight, this.estimatedPose, 1 - weight);
			if (this.estimatedPose.hasNaN())
				reset();
		}
	}

	/**
	 * Calculates the specified weighted sum.
	 * 
	 * @param p1
	 *            the first {@code Pose}
	 * @param w1
	 *            the weight of the first {@code Pose}
	 * @param p2
	 *            the second {@code Pose}
	 * @param w2
	 *            the weight of the second {@code Pose}
	 * @return the weighted sum of the specified {@code Pose}s
	 */
	public static Pose weightedSum(Pose p1, double w1, Pose p2, double w2) {
		if (p1 == null)
			return p2;
		if (p2 == null)
			return p1;
		double a1 = p1.yaw;
		double a2 = p2.yaw;
		if (a1 > a2 + Math.PI)
			a2 += 2 * Math.PI;
		else if (a2 > a1 + Math.PI)
			a1 += 2 * Math.PI;
		return new Pose(p1.x() * w1 + p2.x() * w2, p1.y() * w1 + p2.y() * w2, a1 * w1 + a2 * w2);
	}

}
