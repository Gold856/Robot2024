package hlib.drive;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Stream;

/**
 * A {@code PoseEstimator} estimates the pose of an object based on sample {@code Pose}s.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PoseEstimator {

	/**
	 * The distance threshold for outlier detection (a sample {@code Pose} is considered an outlier and rejected if its
	 * distance from the estimated {@code Pose} is larger than this threshold).
	 */
	protected double distanceThreshold;

	/**
	 * The {@code Pose} representing the pose estimated by this {@code PoseEstimator}.
	 */
	protected Pose estimatedPose = null;

	/**
	 * The {@code PoseErrorTracker} used by this {@code PoseEstimator}.
	 */
	protected PoseErrorTracker errorTracker = new PoseErrorTracker();

	/**
	 * The number of non-{@code null} sample {@code Pose}s that have been given to this {@code PoseEstimator}.
	 */
	protected int nonNullSamples = 0;

	/**
	 * The number of {@code null} sample {@code Pose}s that have been given to this {@code PoseEstimator}.
	 */
	protected int nullSamples = 0;

	/**
	 * The number of {@code Pose}s that have been considered outliers by this {@code PoseEstimator}.
	 */
	protected int outliers = 0;

	/**
	 * The start time (in milliseconds) of this {@code PoseEstimator}.
	 */
	protected long startTime = System.currentTimeMillis();

	/**
	 * The {@code PoseCalculator}s for enhancing the accuracy of the estimated pose based on data from various sources
	 * such as a gyroscope, encoders, etc.
	 */
	protected List<PoseCalculator> poseCalculators = new LinkedList<PoseCalculator>();

	/**
	 * Constructs a {@code PoseEstimator}.
	 * 
	 * @param distanceThreshold
	 *            the distance threshold for outlier detection (a sample {@code Pose} is considered an outlier and
	 *            rejected if its distance from the estimated {@code Pose} is larger than this threshold)
	 */
	public PoseEstimator(double distanceThreshold) {
		this.distanceThreshold = distanceThreshold;
	}

	/**
	 * Returns a {@code Pose} representing the pose estimated by this {@code PoseEstimator}.
	 * 
	 * @return a {@code Pose} representing the pose estimated by this {@code PoseEstimator}
	 */
	public Pose estimatedPose() {
		return estimatedPose;
	}

	/**
	 * Updates this {@code PoseEstimator} based on the specified sample {@code Pose}.
	 * 
	 * @param sample
	 *            a sample {@code Pose}
	 * @return {@code false} if the specified sample {@code Pose} is considered an outlier (because the x- or
	 *         y-coordinate value of the sample {@code Pose} is different by more than the threshold compared to the
	 *         estimated {@code Pose}) and thus rejected; {@code true} if this {@code PoseEstimator} is updated based on
	 *         the specified {@code Pose}
	 */
	public final synchronized boolean update(Pose sample) {
		if (sample != null)
			nonNullSamples++;
		else
			nullSamples++;
		if (isOutlier(sample))
			return false;
		estimatedPose(sample);
		return true;
	}

	/**
	 * Adds {@code PoseCalculator}s for enhancing the accuracy of the estimated pose based on data from various sources
	 * such as a gyroscope, encoders, etc.
	 * 
	 * @param poseCalculators
	 *            {@code PoseCalculator}s for enhancing the accuracy of the estimated pose based on data from various
	 *            sources such as a gyroscope, encoders, etc.
	 */
	public void add(PoseCalculator... poseCalculators) {
		this.poseCalculators.addAll(Arrays.asList(poseCalculators));
	}

	/**
	 * Needs to be invoked periodically in order to enhance the accuracy of the estimated pose based on data from
	 * various sources such as a gyroscope, encoders, etc.
	 */
	public void periodic() {
		if (estimatedPose != null && poseCalculators.size() > 0) {
			Stream<Pose> poses = poseCalculators.stream().map(c -> c.pose(estimatedPose));
			estimatedPose = Pose.avarage(poses.toList().toArray(new Pose[0]));
		}
	}

	/**
	 * Returns the rate (the number of sample {@code Pose}s per second) at which sample {@code Pose}s have been given to
	 * this {@code PoseEstimator}.
	 * 
	 * @return the rate (the number of sample {@code Pose}s per second) at which sample {@code Pose}s have been given to
	 *         this {@code PoseEstimator}
	 */
	public double samplingRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return nonNullSamples / time;
		else
			return 0;
	}

	/**
	 * Returns the rate at which {@code null} sample {@code Pose}s have been given to this {@code PoseEstimator}
	 * 
	 * @return the rate at which {@code null} sample {@code Pose}s have been given to this {@code PoseEstimator}
	 */
	public double samplingFailureRate() {
		double time = 0.001 * (System.currentTimeMillis() - startTime);
		if (time > 0)
			return nullSamples / time;
		else
			return 0;
	}

	/**
	 * Returns the number of outliers (i.e., sample {@code Pose}s that have been rejected by this
	 * {@code PoseEstimator}).
	 * 
	 * @return the number of outliers (i.e., sample {@code Pose}s that have been rejected by this {@code PoseEstimator})
	 */
	public int outliers() {
		return outliers;
	}

	/**
	 * Returns the largest differences (in x- and y-coordinate values as well as orientation values) between the
	 * {@code Pose} estimated by this {@code PoseEstimator} and sample {@code Pose}s.
	 * 
	 * @return the largest differences (in x- and y-coordinate values as well as orientation values) between the
	 *         {@code Pose} estimated by this {@code PoseEstimator} and sample {@code Pose}s
	 */
	public Pose largestPoseInconsistency() {
		return errorTracker.largestPoseError();
	}

	/**
	 * Determines whether or not the specified sample {@code Pose} is an outlier.
	 * 
	 * @param sample
	 *            a sample {@code Pose}
	 * @return {@code true} if either the x- or y-coordinate value of the sample {@code Pose} is different by more than
	 *         the threshold compared to the estimated {@code Pose} maintained by this {@code PoseEstimator};
	 *         {@code false} otherwise
	 */
	protected boolean isOutlier(Pose sample) {
		if (sample == null || this.estimatedPose == null)
			return false;
		if (sample.hasNaN())
			return true;
		Pose error = PoseErrorTracker.error(sample, this.estimatedPose);
		if (Math.abs(error.x()) > distanceThreshold || Math.abs(error.y()) > distanceThreshold) {
			outliers++;
			return true;
		} else
			return false;
	}

	/**
	 * Updates the estimated {@code Pose} based on the specified sample {@code Pose}.
	 * 
	 * @param sample
	 *            a sample {@code Pose}
	 */
	protected void estimatedPose(Pose sample) {
		if (sample != null) {
			errorTracker.update(sample, this.estimatedPose);
			this.estimatedPose = sample;
		}
	}

}
