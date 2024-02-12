package frc.robot.subsystems;

import java.util.Collection;
import java.util.EnumSet;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;

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
public class LimeLightSubsystem extends SubsystemBase {

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

		/**
		 * Determines whether or not any coordinate or yaw value of this {@code Pose2d}
		 * is NaN.
		 * 
		 * @return {@code true} if any coordinate or yaw value of this {@code Pose2d} is
		 *         NaN; {@code false} otherwise
		 */
		public boolean hasNaN() {
			return Double.isNaN(getX()) || Double.isNaN(getY())
					|| Double.isNaN(getRotation().getRadians());
		}

	}

	protected static final Pose2d DEFAULT_POSE = new Pose(0, 0, 0);

	/**
	 * The most recent botpose data obtained from LimeLight.
	 */
	protected TimestampedDoubleArray m_botpose;

	/**
	 * The most recent data about recognized AprilTags.
	 */
	protected TimestampedObject<Map<String, double[]>> m_tags;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public LimeLightSubsystem() {
		subscribe("limelight", "botpose", new double[6], event -> changedBotPose(event));
		subscribe("limelight", "json", "", event -> changedJson(event));
	}

	/**
	 * Returns a {@code Pose2d} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose2d} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose2d estimatedPose() {
		if (m_botpose == null)
			return null;
		var pose = new Pose(m_botpose.value[0], m_botpose.value[1], m_botpose.value[5]);
		return pose.hasNaN() || pose.equals(DEFAULT_POSE) ? null : pose;
	}

	public Pose2d getPose(String tagID) {
		if (m_tags == null)
			return null;
		var v = m_tags.value.get(tagID);
		return new Pose(v[0], v[1], v[2]);
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
		} catch (Exception e) {
			m_botpose = null;
			e.printStackTrace();
		}
		return m_botpose;
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
		} catch (Exception e) {
			m_tags = null;
			e.printStackTrace();
		}
		return m_tags;
	}

	/**
	 * Returns the distance to the specified target position
	 * 
	 * @param targetPosition the target position
	 * @return the distance to the specified target position
	 */
	public double getDistance(Translation2d targetPosition) {
		return getDistance(estimatedPose(), targetPosition);
	}

	/**
	 * Returns the distance to the specified target AprilTag
	 * 
	 * @param targetPosition the ID of the AprilTag
	 * @return the distance to the specified target AprilTag
	 */
	public Double getDistance(String tagID) {
		var pose = getPose(tagID);
		return pose == null ? null : pose.getTranslation().getNorm();
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
		return estimatedPose() == null ? null : getRotation(estimatedPose(), targetPosition);
	}

	/**
	 * Calculates the target {@code Pose2d} based on the current pose of the robot
	 * and the specified target position and distance to the target.
	 * 
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance to the target
	 * @return the target {@code Pose2d} calculated based on the current pose of the
	 *         robot
	 *         and the specified target position and distance to the target
	 * @throw UnsupportedOperationException if the current robot pose and
	 *        the target are at the same location
	 */
	public Pose2d getTargetPose(Translation2d target, double distanceToTarget) {
		return estimatedPose() == null ? null : getTargetPose(estimatedPose(), target, distanceToTarget);
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified AprilTag
	 * 
	 * @param targetPosition the ID of the AprilTag
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified AprilTag
	 */
	public Rotation2d getRotation(String tagID) {
		var pose = getPose(tagID);
		return pose == null ? null
				: pose.getTranslation().getAngle()
						.minus(Rotation2d.fromDegrees(90));
	}

	/**
	 * Returns the {@code Rotation2d} that is needed to turn the robot toward the
	 * specified AprilTag
	 * 
	 * @param targetPosition the ID of the AprilTag
	 * @return the {@code Rotation2d} that is needed to turn the robot toward the
	 *         specified AprilTag
	 */
	public Rotation2d getRotationToDetectedTags() {
		if (m_tags == null)
			return null;
		Pose2d pose = average(m_tags.value.values());
		return pose == null ? null
				: pose.getTranslation().getAngle()
						.minus(Rotation2d.fromDegrees(90));
	}

	public static Pose2d average(Collection<double[]> values) {
		if (values == null || values.size() == 0)
			return null;
		Translation2d t = null;
		Rotation2d r = null;
		var i = values.iterator();
		while (i.hasNext()) {
			var v = i.next();
			if (t == null) {
				t = new Translation2d(v[0], v[1]);
				r = Rotation2d.fromDegrees(v[2]);
			} else {
				t = t.plus(new Translation2d(v[0], v[1]));
				r = r.plus(Rotation2d.fromDegrees(v[2]));
			}
		}
		return new Pose2d(t.div(values.size()), r.div(values.size()));
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

	public static Rotation2d getRotation(Pose2d pose, Translation2d target) {
		return target.minus(pose.getTranslation()).getAngle().minus(pose.getRotation());
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

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	protected void recordPose(String entryName, Pose2d value) {
	}

	/**
	 * Records the specified value in the specified entry in a {@code NetworkTable}.
	 * 
	 * @param entryName the name of the entry
	 * @param value     the value to record
	 */
	protected void recordString(String entryName, String value) {
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
			var i = e.path("t6t_rs").elements();
			double v1 = i.next().asDouble();
			i.next();
			double v2 = i.next().asDouble();
			i.next();
			i.next();
			double v3 = i.next().asDouble();
			var v = new double[] { v1, v2, v3 };
			m.put(e.path("fID").toString(), v);
		});
		return m;
	}

}
