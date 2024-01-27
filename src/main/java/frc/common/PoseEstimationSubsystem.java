package frc.common;

import java.io.File;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import hlib.drive.AprilTagMap;
import hlib.drive.Pose;
import hlib.drive.PoseCalculator;
import hlib.drive.PoseEstimator;
import hlib.drive.PoseEstimatorWeighted;

/**
 * The purpose of the {@code PoseEstimationSubsystem} is to provide the pose of
 * the robot, each AprilTag, as well as others of interest. For stationary
 * objects such AprilTags, it stores the corresponding {@code Pose}s and provide
 * them as needed.
 * For a moving object such as the robot, it estimates the pose based on a
 * variety of sources including LimeLight as well as encoders and sensors
 * attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimationSubsystem extends AprilTagSubsystem {

	/**
	 * The {@code PoseEstimator} for estimating the pose of the robot.
	 */
	protected PoseEstimator m_poseEstimator = new PoseEstimatorWeighted(1.0, 10, 0.1);

	/**
	 * A {@code Map} that maps the ID of each AprilTag to the {@code Pose} of that
	 * AprilTag.
	 */
	Map<Integer, Pose> m_aprilTagPoses = new TreeMap<Integer, Pose>();

	/**
	 * The {@code PoseCalculator}s for enhancing the accuracy of the estimated pose
	 * based on data from various sources
	 * such as a gyroscope, encoders, etc.
	 */
	protected Map<String, PoseCalculator> m_poseCalculators = new TreeMap<String, PoseCalculator>();

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystem() {
		super();
		try {
			var m = new AprilTagMap(RobotContainer.s_deployPath + File.separator + "2024LimeLightMap.fmap");
			m.forEach((k, v) -> m_aprilTagPoses.put(k, AprilTagMap.toPose(v)));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Returns the {@code Pose} of the specified AprilTag.
	 * 
	 * @param tagID the ID of the AprilTag
	 * @return the {@code Pose} of the specified AprilTag ({@code null} if there is
	 *         no such AprilTag)
	 */
	public Pose aprilTagPose(int tagID) {
		return m_aprilTagPoses.get(tagID);
	}

	/**
	 * Returns the only {@code PoseEstimationSubsystem} instantiated in compliance
	 * with the singleton design pattern.
	 * 
	 * @return the only {@code PoseEstimationSubsystem} instantiated in compliance
	 *         with the singleton design pattern
	 */
	public static PoseEstimationSubsystem get() {
		return (PoseEstimationSubsystem) s_subsystem;
	}

	/**
	 * Is invoked periodically (every 20 ms approximately).
	 */
	@Override
	public void periodic() {
		m_poseEstimator.update(m_poseCalculators.values());
	}

	/**
	 * Is invoked whenever there is a change in the {@code NetworkTable} entry
	 * representing the pose of the robot.
	 */
	protected TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		var botpose = super.changedBotPose(event);
		if (botpose != null) {
			m_poseEstimator.update(new Pose(botpose.value[0], botpose.value[1], botpose.value[5] * Math.PI / 180));
			var estimatedPose = m_poseEstimator.estimatedPose();
			visionTable.getEntry("Pose Estimated").setString("" + estimatedPose());
			if (estimatedPose != null)
				visionTable.getEntry("Pose2D'").setDoubleArray(toPose2DAdvantageScope(estimatedPose.x(),
						estimatedPose.y(), estimatedPose.yawInDegrees()));
		}
		return botpose;
	}

	/**
	 * Returns a {@code Pose} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose estimatedPose() {
		var pose = m_poseEstimator.estimatedPose();
		return pose == null || pose.hasNaN() || pose.equals(Pose.DEFAULT_POSE) ? null : pose;
	}

	/**
	 * Returns a {@code Pose} obtained from the specified {@code Pose2d}.
	 * 
	 * @param a {@code Pose2d}
	 * @return a {@code Pose} obtained from the specified {@code Pose2d}
	 */
	public static Pose toPose(Pose2d pose) {
		return new Pose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
	}

	/**
	 * Adds a {@code Supplier<Pose>} which can provide {@code Pose}s obtained from
	 * some sources such as a gyroscope,
	 * encoders, etc. in order to enhance the accuracy of the pose estimated by this
	 * {@code PoseEstimator}.
	 * 
	 * @param label
	 *                     a label associated with the specifiled
	 *                     {@code Supplier<Pose>}
	 * @param poseSupplier
	 *                     a {@code Supplier<Pose>} which can provide {@code Pose}s
	 *                     obtained from some sources such as a
	 *                     gyroscope, encoders, etc.
	 */
	public void addPoseSupplier(String label, Supplier<Pose> poseSupplier) {
		this.m_poseCalculators.put(label, new PoseCalculator() {

			Pose previous = null;

			@Override
			public Pose pose(Pose pose) {
				var current = poseSupplier.get();
				visionTable.getEntry(label).setDoubleArray(PoseEstimationSubsystem.toPose2DAdvantageScope(current.x(),
						current.y(), current.yawInDegrees()));
				if (this.previous == null || pose == null) {
					this.previous = current;
					return pose;
				}
				var refined = pose.move(this.previous, current);
				this.previous = current;
				return refined;
			}

		});
	}

}
