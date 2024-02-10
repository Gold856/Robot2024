package frc.robot.subsystems;

import java.io.File;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The purpose of the {@code PoseEstimationSubsystem} is to provide the pose of
 * the robot, each AprilTag, as well as others of interest. For stationary
 * objects such AprilTags, it stores the corresponding {@code Pose2d}s and
 * provide
 * them as needed.
 * For a moving object such as the robot, it estimates the pose based on a
 * variety of sources including LimeLight as well as encoders and sensors
 * attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class PoseEstimationSubsystem extends SubsystemBase {

	/**
	 * A wrapper class for simple construction and string representation of
	 * {@code Pose2d} instances.
	 */
	public static class Pose extends Pose2d {

		/**
		 * Constrcuts a {@code Pose}
		 * 
		 * @param x            the x-coordinate value
		 * @param y            the y-coordinate value
		 * @param yawInDegrees the yaw in degrees
		 */
		public Pose(double x, double y, double yawInDegrees) {
			super(x, y, Rotation2d.fromDegrees(yawInDegrees));
		}

		/**
		 * Returns a string representation of this {@code Pose}.
		 */
		@Override
		public String toString() {
			return String.format("[%.1f, %.1f, %.1f degrees]", getX(), getY(), getRotation().getDegrees());
		}

	}

	/**
	 * A {@code PoseCalculator} calculates the pose of an object based on the pose
	 * of that object at an earlier time and some
	 * changes in that object observed via some sources such as a gyroscope,
	 * encoders, etc.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public interface PoseCalculator {

		/**
		 * Calculates the current pose of an object based on the pose of that object at
		 * an earlier time and some changes in
		 * that object observed via some sources such as a gyroscope, encoders, etc.
		 * 
		 * @param pose
		 *             a {@code Pose} representing the pose of the object at an earlier
		 *             time
		 * @return a {@code Pose} representing the pose calculated by this
		 *         {@code PoseCalculator}
		 */
		public Pose2d pose(Pose2d pose);

	}

	/**
	 * A {@code PoseEstimator} estimates the pose of an object based on sample
	 * {@code Pose2d}s.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public class PoseEstimator {

		/**
		 * The distance threshold for outlier detection (a sample {@code Pose2d} is
		 * considered an outlier and rejected if its
		 * distance from the estimated {@code Pose2d} is larger than this threshold).
		 */
		protected double distanceThreshold;

		/**
		 * The {@code Pose2d} representing the pose estimated by this
		 * {@code PoseEstimator}.
		 */
		protected Pose2d estimatedPose = null;

		/**
		 * The number of {@code Pose2d}s that have been considered outliers by this
		 * {@code PoseEstimator}.
		 */
		protected int outliers = 0;

		/**
		 * The start time (in milliseconds) of this {@code PoseEstimator}.
		 */
		protected long startTime = System.currentTimeMillis();

		/**
		 * The number of consecutive rejections of sample {@code Pose2d}s needed to
		 * reset this {@code PoseEstimator}.
		 */
		int rejectionLimit;

		/**
		 * The number of consecutive sample {@code Pose2d}s that have been rejected as
		 * outliers.
		 */
		int rejections = 0;

		/**
		 * The weight applied to each sample {@code Pose2d}.
		 */
		protected double weight;

		/**
		 * Constructs a {@code PoseEstimator}.
		 * 
		 * @param distanceThreshold
		 *                          the distance threshold for outlier detection (a
		 *                          sample {@code Pose2d} is considered an outlier and
		 *                          rejected if its distance from the estimated
		 *                          {@code Pose2d} is larger than this threshold)
		 * @param rejectionLimit
		 *                          the number of rejections needed to reset the
		 *                          {@code PoseEstimatorWeighted}
		 * @param weight
		 *                          the weight of each sample {@code Pose2d}
		 */
		public PoseEstimator(double distanceThreshold, int rejectionLimit, double weight) {
			this.distanceThreshold = distanceThreshold;
			this.rejectionLimit = rejectionLimit;
			this.weight = weight;
		}

		/**
		 * Returns a {@code Pose2d} representing the pose estimated by this
		 * {@code PoseEstimator}.
		 * 
		 * @return a {@code Pose2d} representing the pose estimated by this
		 *         {@code PoseEstimator}
		 */
		public Pose2d estimatedPose() {
			return estimatedPose;
		}

		/**
		 * Updates this {@code PoseEstimator} based on the specified sample
		 * {@code Pose2d}.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 * @return {@code false} if the specified sample {@code Pose2d} is considered an
		 *         outlier (because the x- or
		 *         y-coordinate value of the sample {@code Pose2d} is different by more
		 *         than the threshold compared to the
		 *         estimated {@code Pose2d}) and thus rejected; {@code true} if this
		 *         {@code PoseEstimator} is updated based on
		 *         the specified {@code Pose2d}
		 */
		public final synchronized boolean update(Pose2d sample) {
			if (isOutlier(sample))
				return false;
			estimatedPose(sample);
			return true;
		}

		/**
		 * Returns the number of outliers (i.e., sample {@code Pose2d}s that have been
		 * rejected by this
		 * {@code PoseEstimator}).
		 * 
		 * @return the number of outliers (i.e., sample {@code Pose2d}s that have been
		 *         rejected by this {@code PoseEstimator})
		 */
		public int outliers() {
			return outliers;
		}

		/**
		 * Determines whether or not the specified sample {@code Pose2d} is an outlier.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 * @return {@code true} if either the x- or y-coordinate value of the sample
		 *         {@code Pose2d} is different by more than
		 *         the threshold compared to the estimated {@code Pose2d} maintained by
		 *         this {@code PoseEstimator};
		 *         {@code false} otherwise
		 */
		protected boolean isOutlier(Pose2d sample) {
			if (sample == null || this.estimatedPose == null)
				return false;
			if (hasNaN(sample))
				return true;
			Pose2d error = error(sample, this.estimatedPose);
			if (Math.abs(error.getX()) > distanceThreshold || Math.abs(error.getY()) > distanceThreshold) {
				outliers++;
				if (++rejections > rejectionLimit)
					reset();
				return true;
			} else {
				if (sample != null && this.estimatedPose != null)
					rejections = 0;
				return false;
			}
		}

		/**
		 * /** Resets this {@code PoseEstimator}.
		 */
		protected void reset() {
			System.out.println("resetting the pose estimator...");
			estimatedPose = null;
			rejections = 0;
		}

		/**
		 * Updates the estimated {@code Pose2d} based on the specified sample
		 * {@code Pose2d}.
		 * 
		 * @param sample
		 *               a sample {@code Pose2d}
		 */
		public void estimatedPose(Pose2d sample) {
			if (sample != null) {
				this.estimatedPose = weightedSum(sample, weight, this.estimatedPose, 1 - weight);
				if (hasNaN(this.estimatedPose))
					reset();
			}
		}

		/**
		 * Calculates the discrepancies between the specified {@code Pose2d}s.
		 * 
		 * @param pose
		 *                  a {@code Pose2d}
		 * @param reference
		 *                  a reference {@code Pose2d} for comparison
		 * @return a {@code Pose2d} representing the difference between the
		 *         {@code Pose2d}s
		 *         ({@code null} if either of the given
		 *         {@code Pose2d}s is {@code null})
		 */
		public static Pose2d error(Pose2d pose, Pose2d reference) {
			if (pose == null || reference == null)
				return null;
			return new Pose2d(pose.getX() - reference.getX(), pose.getY() - reference.getY(),
					pose.getRotation().minus(reference.getRotation()));
		}

		/**
		 * Calculates the specified weighted sum.
		 * 
		 * @param p1
		 *           the first {@code Pose2d}
		 * @param w1
		 *           the weight of the first {@code Pose2d}
		 * @param p2
		 *           the second {@code Pose2d}
		 * @param w2
		 *           the weight of the second {@code Pose2d}
		 * @return the weighted sum of the specified {@code Pose2d}s
		 */
		public static Pose2d weightedSum(Pose2d p1, double w1, Pose2d p2, double w2) {
			if (p1 == null)
				return p2;
			if (p2 == null)
				return p1;
			double a1 = p1.getRotation().getRadians();
			double a2 = p2.getRotation().getRadians();
			if (a1 > a2 + Math.PI)
				a2 += 2 * Math.PI;
			else if (a2 > a1 + Math.PI)
				a1 += 2 * Math.PI;
			return new Pose2d(p1.getX() * w1 + p2.getX() * w2, p1.getY() * w1 + p2.getY() * w2,
					Rotation2d.fromRadians(a1 * w1 + a2 * w2));
		}

		/**
		 * Determines whether or not any coordinate or yaw value of this {@code Pose2d}
		 * is NaN.
		 * 
		 * @return {@code true} if any coordinate or yaw value of this {@code Pose2d} is
		 *         NaN; {@code false} otherwise
		 */
		public static boolean hasNaN(Pose2d pose) {
			return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())
					|| Double.isNaN(pose.getRotation().getRadians());
		}

		/**
		 * Updates the {@code Pose} estimated by this {@code PoseEstimator} based on the
		 * specified {@code PoseCalculator}s.
		 * 
		 * @param poseCalculators
		 *                        {@code PoseCalculator}s
		 */
		public void update(Collection<PoseCalculator> poseCalculators) {
			if (poseCalculators.size() > 0) {
				Stream<Pose2d> poses = poseCalculators.stream().map(c -> c.pose(estimatedPose));
				estimatedPose = average(poses.toList().toArray(new Pose2d[0]));
			}
		}

		/**
		 * Returns the average of the specified {@code Pose2d}s.
		 * 
		 * @param poses
		 *              {@code Pose}s
		 * @return the average of the specified {@code Pose2d}s
		 */
		public static Pose2d average(Pose2d... poses) {
			if (poses == null || poses.length == 0)
				return null;
			double x = 0;
			double y = 0;
			double yaw = 0;
			int count = 0;
			for (var pose : poses) {
				if (pose != null) {
					x += pose.getX();
					y += pose.getY();
					yaw += pose.getRotation().getDegrees();
					count++;
				}
			}
			if (count == 0)
				return null;
			return new Pose2d(x / poses.length, y / poses.length, Rotation2d.fromDegrees(yaw / poses.length));
		}
	}

	/**
	 * A {@code AprilTagMap} maps the ID of each AprilTag to the 3D transform
	 * representing the pose of that AprilTag.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public class AprilTagMap extends HashMap<Integer, double[]> {

		/**
		 * The automatically generated serial version UID.
		 */
		private static final long serialVersionUID = -7392062114679722757L;

		/**
		 * Constructs an {@code AprilTagMap} by parsing the specified JSON file.
		 * 
		 * @param fileName
		 *                 the name of the file
		 */
		public AprilTagMap(String fileName) {
			try {
				JsonNode root = new ObjectMapper().readTree(new File(fileName));
				JsonNode poses = root.path("fiducials");
				Iterator<JsonNode> i = poses.elements();
				while (i.hasNext()) {
					JsonNode n = i.next();
					int tagID = n.path("id").asInt();
					Iterator<JsonNode> j = n.path("transform").elements();
					double[] transform = new double[] { j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble(), j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble(), j.next().asDouble(), j.next().asDouble(), j.next().asDouble(),
							j.next().asDouble() };
					put(tagID, transform);
				}
			} catch (Exception e1) {
				e1.printStackTrace();
			}
			System.out.println(size() + " tags read from \"" + fileName + "\".");
		}

		/**
		 * Constructs a {@code Pose2d} from the specified 3D transformation.
		 * 
		 * @param transform
		 *                  a {@code double} array representing a 3D transformation
		 * @return a {@code Pose2d} constructed from the specified 3D transformation
		 */
		public static Pose2d toPose(double[] transform) {
			double mxx = transform[0];
			double mxy = transform[1];
			double tx = transform[3];
			double ty = transform[7];
			return new Pose2d(tx, ty, Rotation2d.fromRadians(-Math.atan2(mxy, mxx)));
		}

	}

	private static final Pose2d DEFAULT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

	/**
	 * The only instance of the {@code PoseEstimationSubsystem} class.
	 */
	public static PoseEstimationSubsystem s_subsystem;

	/**
	 * The most recent botpose data obtained from LimeLight.
	 */
	protected TimestampedDoubleArray m_botpose;

	/**
	 * The most recent data about recognized AprilTags.
	 */
	protected TimestampedObject<Map<String, double[]>> m_tags;

	/**
	 * The {@code PoseEstimator} for estimating the pose of the robot.
	 */
	protected PoseEstimator m_poseEstimator = new PoseEstimator(1.0, 10, 0.1);

	/**
	 * A {@code Map} that maps the ID of each AprilTag to the {@code Pose2d} of that
	 * AprilTag.
	 */
	protected Map<Integer, Pose2d> m_aprilTagPoses = new TreeMap<Integer, Pose2d>();

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystem() {
		if (s_subsystem != null) // singleton
			try {
				throw new Exception("AprilTag subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		s_subsystem = this;
		subscribe("limelight", "botpose", new double[6], event -> changedBotPose(event));
		subscribe("limelight", "json", "", event -> changedJson(event));
		try {
			var m = new AprilTagMap(Filesystem.getDeployDirectory() + File.separator + "2024LimeLightMap.fmap");
			m.forEach((k, v) -> m_aprilTagPoses.put(k, AprilTagMap.toPose(v)));
		} catch (Exception e) {
			e.printStackTrace();
		}

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
	 * Returns a {@code Pose2d} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose2d} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose2d estimatedPose() {
		var pose = m_poseEstimator.estimatedPose();
		return pose == null || PoseEstimator.hasNaN(pose) || pose.equals(DEFAULT_POSE) ? null : pose;
	}

	/**
	 * Returns, for each detected AprilTag, the distance in meters to that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the distance in
	 *         meters to that AprilTag
	 */
	public Map<Integer, Double> getDistancesToDetectedTags() {
		var pose = m_poseEstimator.estimatedPose();
		return pose == null ? Map.of() : getDistancesToDetectedTags(pose);
	}

	/**
	 * Returns, for each detected AprilTag, the rotation needed for the robot to
	 * face toward that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the rotation
	 *         needed for the robot face toward that AprilTag
	 */
	public Map<Integer, Rotation2d> getRotationsToDetectedTags() {
		var pose = m_poseEstimator.estimatedPose();
		return pose == null ? Map.of() : getRotationsToDetectedTags(pose);
	}

	/**
	 * Returns the most recent botpose data obtained from LimeLight.
	 * 
	 * @return the most recent botpose data obtained from LimeLight
	 */
	public TimestampedDoubleArray botpose() {
		return m_botpose;
	}

	/**
	 * Returns the most recent data about recognized AprilTags.
	 * 
	 * @return a {@code Map} that maps the ID of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the orientation in degrees of the
	 *         displacement.
	 */
	public TimestampedObject<Map<String, double[]>> tags() {
		return m_tags;
	}

	/**
	 * Adds the specified {@code Listener} to respond to the changes in the
	 * specified topic in the specified table.
	 * 
	 * @param tableName    the name of the table
	 * @param topicName    the name of the topic
	 * @param defaultValue the default value used when no value was obtained
	 *                     regarding the topic
	 * @param listener     a {@code Consumer} responsible for responding to the
	 *                     changes in the specified topic
	 */
	public static void subscribe(String tableName, String topicName, String defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable t = i.getTable(tableName);
		var s = t.getStringTopic(topicName).subscribe(defaultValue);
		i.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	/**
	 * Adds the specified {@code Listener} to respond to the changes in the
	 * specified topic in the specified table.
	 * 
	 * @param tableName    the name of the table
	 * @param topicName    the name of the topic
	 * @param defaultValue the default value used when no value was obtained
	 *                     regarding the topic
	 * @param listener     a {@code Consumer} responsible for responding to the
	 *                     changes in the specified topic
	 */
	public static void subscribe(String tableName, String topicName, double[] defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable t = i.getTable(tableName);
		var s = t.getDoubleArrayTopic(topicName).subscribe(defaultValue);
		i.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	/**
	 * Is invoked whenever the "botpose" entry in the "limelight" table changes.
	 * 
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "botpose" entry in the "limelight" table
	 * @return a {@code TimestampedDoubleArray} representing the pose of the robot
	 *         in terms of the x and y-coordinate values and the yaw value (the
	 *         orientation relative to the positive x-axis) in
	 *         degrees
	 */
	protected TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			m_botpose = new TimestampedDoubleArray(v.getTime(), v.getServerTime(), v.getDoubleArray());
			if (m_botpose != null) {
				var pose = new Pose2d(m_botpose.value[0], m_botpose.value[1],
						Rotation2d.fromDegrees(m_botpose.value[5]));
				if (pose.getX() != 0 || pose.getY() != 0 || pose.getRotation().getDegrees() != 0)
					m_poseEstimator.update(pose);
			}
			return m_botpose;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
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
		try {
			var v = event.valueData.value;
			var m = toMap(v.getString());
			m_tags = new TimestampedObject<Map<String, double[]>>(v.getTime(), v.getServerTime(), m);
			return m_tags;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * Parses the specified JSON expression.
	 * 
	 * @param s a {@code String} containing a JSON expression
	 * @return a {@code Map} that maps the ID of each recognized AprilTag to a
	 *         {@code double} array representing the displacement from the camera to
	 *         that AprilTag. Each {@code double} array contains the x and
	 *         y-coordinate values and the yaw value (i.e., the orientation relative
	 *         to the positive x-axis) in degrees of the displacement
	 * @throws JsonMappingException    if a parsing error occurs
	 * @throws JsonProcessingException if a parsing error occurs
	 */
	protected static Map<String, double[]> toMap(String s) throws JsonMappingException, JsonProcessingException {
		JsonNode n = new ObjectMapper().readTree(s);
		n = n.path("Results").path("Fiducial");
		var m = new TreeMap<String, double[]>();
		n.forEach(e -> {
			var i = e.path("t6t_cs").elements();
			double v1 = i.next().asDouble();
			i.next();
			double v2 = i.next().asDouble();
			i.next();
			double v3 = i.next().asDouble();
			var v = new double[] { v1, v2, v3 };
			m.put(e.path("fID").toString(), v);
		});
		return m;
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified target position
	 * 
	 * @param targetPosition the target position
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified target position
	 */
	public Rotation2d getRotation(Translation2d targetPosition) {
		return getRotation(this.estimatedPose(), targetPosition);
	}

	public static Rotation2d getRotation(Pose2d pose, Translation2d target) {
		return target.minus(pose.getTranslation()).getAngle().minus(pose.getRotation());
	}

	/**
	 * Returns the distance from the specified {@code Pose2d} to the specified
	 * target position.
	 * 
	 * @param pose   a {@code Pose2d}
	 * @param target a target position
	 * @return the distance from the specified {@code Pose2d} to the specified
	 *         target position
	 */
	public static double getDistance(Pose2d pose, Translation2d target) {
		return pose.getTranslation().getDistance(target);
	}

	/**
	 * Returns, for each detected AprilTag, the distance in meters to that AprilTag
	 * from the specified {@code Pose2d}
	 * 
	 * @param pose a {@code Pose2d}
	 * @return a {@code Map} containing, for each detected AprilTag, the distance in
	 *         meters to that AprilTag from the specified {@code Pose2d}
	 */
	public Map<Integer, Double> getDistancesToDetectedTags(Pose2d pose) {
		var m = new TreeMap<Integer, Double>();
		if (m_tags == null)
			return m;
		for (String s : m_tags.value.keySet()) {
			try {
				int i = Integer.parseInt(s);
				var p = m_aprilTagPoses.get(i);
				m.put(i, getDistance(pose, p.getTranslation()));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		return m;
	}

	/**
	 * Returns, for each detected AprilTag, the rotation needed for the specified
	 * {@code Pose2d} to face toward that AprilTag.
	 * 
	 * @return a {@code Map} containing, for each detected AprilTag, the rotation
	 *         needed for the specified {@code Pose2d} face toward that AprilTag
	 */
	public Map<Integer, Rotation2d> getRotationsToDetectedTags(Pose2d pose) {
		var m = new TreeMap<Integer, Rotation2d>();
		if (m_tags == null)
			return m;
		for (String s : m_tags.value.keySet()) {
			try {
				int i = Integer.parseInt(s);
				var p = m_aprilTagPoses.get(i);
				m.put(i, getRotation(pose, p.getTranslation()));
			} catch (Exception e) {
			}
		}
		return m;
	}

	public static Transform2d getTransform(Pose2d pose, Translation2d target) {
		return new Transform2d(pose, getTargetPose(pose, target, 0));
	}

	/**
	 * Calculates the target {@code Pose2d} based on the specified current
	 * {@code Pose2d}, target position, and distance to the target.
	 * 
	 * @param currentPose      the current {@code Pose2d} of the robot
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance to the target
	 * @return the target {@code Pose2d} calculated based on the specified current
	 *         {@code Pose2d}, target position, and distance to the target
	 * @throw UnsupportedOperationException if the specified {@code Pose2d} and
	 *        target are at the same location
	 */
	public static Pose2d getTargetPose(Pose2d pose, Translation2d target, double distanceToTarget) {
		Translation2d diff = target.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			throw new UnsupportedOperationException();
		return new Pose2d(pose.getTranslation().plus(diff.times(1 - distanceToTarget / diff.getNorm())),
				diff.getAngle());
	}

	public Map<Integer, Pose2d> aprilTagPoses() {
		return this.m_aprilTagPoses;
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordPose(String entryName, Pose2d value) {
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	public void recordString(String entryName, String value) {
	}

}
