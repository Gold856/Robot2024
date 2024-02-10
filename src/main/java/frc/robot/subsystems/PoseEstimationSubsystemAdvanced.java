package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedObject;

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
public class PoseEstimationSubsystemAdvanced extends PoseEstimationSubsystem {

	/**
	 * The path to the "deploy" directory in the project.
	 */
	public final static String s_deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
			+ "deploy";
	/**
	 * The {@code NetworkTable} named "vision" which is used by this
	 * {@code AprilTagSubsystem}.
	 */
	protected NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("AdvantageScope");

	/**
	 * The {@code PoseCalculator}s for enhancing the accuracy of the estimated pose
	 * based on data from various sources
	 * such as a gyroscope, encoders, etc.
	 */
	protected Map<String, PoseCalculator> m_poseCalculators = new TreeMap<String, PoseCalculator>();

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystemAdvanced() {
		super();
		try {
			var m = new AprilTagMap(s_deployPath + File.separator + "2024LimeLightMap.fmap");
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
	public Pose2d aprilTagPose(int tagID) {
		return m_aprilTagPoses.get(tagID);
	}

	/**
	 * Returns the only {@code PoseEstimationSubsystem} instantiated in compliance
	 * with the singleton design pattern.
	 * 
	 * @return the only {@code PoseEstimationSubsystem} instantiated in compliance
	 *         with the singleton design pattern
	 */
	public static PoseEstimationSubsystemAdvanced get() {
		return (PoseEstimationSubsystemAdvanced) s_subsystem;
	}

	/**
	 * Is invoked periodically (every 20 ms approximately).
	 */
	@Override
	public void periodic() {
		m_poseEstimator.update(m_poseCalculators.values());
		var pose = estimatedPose();
		visionTable.getEntry("Pose Estimated").setString("" + pose);
		if (pose != null)
			visionTable.getEntry("BotPose'").setDoubleArray(toPose2DAdvantageScope(pose.getX(),
					pose.getY(), pose.getRotation().getDegrees()));
	}

	/**
	 * Is invoked whenever there is a change in the {@code NetworkTable} entry
	 * representing the pose of the robot.
	 */
	protected TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		var botpose = super.changedBotPose(event);
		if (botpose != null) {
			visionTable.getEntry("BotPose")
					.setDoubleArray(toPose2DAdvantageScope(m_botpose.value[0], m_botpose.value[1], m_botpose.value[5]));
		}
		return botpose;
	}

	/**
	 * Is invoked whenever the "json" entry in the "limelight" table changes.
	 * 
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "json" entry in the "limelight" table
	 * @return a {@code TimestampedObject} containing a {@code Map} that maps the ID
	 *         of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the yaw value (i.e., the orientation relative
	 *         to the positive x-axis) in degrees of the displacement
	 */
	protected TimestampedObject<Map<String, double[]>> changedJson(NetworkTableEvent event) {
		var tags = super.changedJson(event);
		if (tags != null)
			visionTable.getEntry("Json").setString(toString(tags.value));
		return tags;
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
	public void addPoseSupplier(String label, Supplier<Pose2d> poseSupplier) {
		this.m_poseCalculators.put(label, new PoseCalculator() {

			Pose2d previous = null;

			@Override
			public Pose2d pose(Pose2d pose) {
				var current = poseSupplier.get();
				recordPose(label, current);
				if (this.previous == null || pose == null) {
					this.previous = current;
					return pose;
				}
				var refined = pose.plus(current.minus(this.previous));
				this.previous = current;
				return refined;
			}

		});
	}

	/**
	 * Constructs a {@code double} array for AdvantageScope from the specified
	 * values.
	 * 
	 * @param x            the x-coordinate value of the pose of the robot
	 * @param y            the y-coordinate value of the pose of the robot
	 * @param yawInDegrees the yaw value (i.e., the orientation relative to the
	 *                     positive x-axis) of the pose of the robot
	 * @return a {@code double} array for AdvantageScope regarding the pose of the
	 *         robot.
	 */
	static double[] toPose2DAdvantageScope(double x, double y, double yawInDegrees) {
		return new double[] { x + 8.27, y + 4.1, yawInDegrees * Math.PI / 180 };
	}

	/**
	 * Returns a string representation of the specified {@code Map}.
	 * 
	 * @param m a {@code Map} that maps the ID of each recognized AprilTag to a
	 *          {@code double} array representing the displacement from the camera
	 *          to that AprilTag. Each {@code double} array contains the x and
	 *          y-coordinate values and the yaw value (i.e., the orientation
	 *          relative to the positive x-axis) in degrees of the displacement
	 * @return a string representation of the specified {@code Map}
	 */
	String toString(Map<String, double[]> m) {
		if (m != null && m.size() > 0) {
			Stream<String> stream = m.entrySet().stream().map(e -> e.getKey() + "=" + Arrays.toString(e.getValue()));
			return stream.reduce((e1, e2) -> e1 + "," + e2).get();
		}
		return "";
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordPose(String entryName, Pose2d value) {
		if (value != null && !(value instanceof Pose))
			value = new Pose(value.getX(), value.getY(), value.getRotation().getDegrees());
		if (value == null)
			visionTable.getEntry(entryName).setDoubleArray(new double[0]);
		else
			visionTable.getEntry(entryName).setDoubleArray(toPose2DAdvantageScope(value.getX(),
					value.getY(), value.getRotation().getDegrees()));
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordString(String entryName, String value) {
		visionTable.getEntry(entryName).setString(value);
	}

}
