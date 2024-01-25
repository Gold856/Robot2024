package frc.common;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code AprilTagSubsystem} class implements a subsystem for tracking and
 * updating the positions and other data related to AprilTags.
 * {@code AprilTagSubsystem} follows the singleton design pattern to ensure that
 * only one instance of the subsystem exists.
 */
public class AprilTagSubsystem extends SubsystemBase {

	/**
	 * The only instance of the {@code AprilTagSubsystem} class.
	 */
	public static AprilTagSubsystem s_subsystem;

	/**
	 * The most recent botpose data obtained from LimeLight.
	 */
	private TimestampedDoubleArray m_botpose;

	/**
	 * The most recent data about recognized AprilTags.
	 */
	private TimestampedObject<Map<String, double[]>> m_tags;

	/**
	 * The {@code NetworkTable} named "vision" which is used by this
	 * {@code AprilTagSubsystem}.
	 */
	protected NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

	/**
	 * Constructs an {@code AprilTagSubsystem}.
	 */
	public AprilTagSubsystem() {
		if (s_subsystem != null) // singleton
			try {
				throw new Exception("AprilTag subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		s_subsystem = this;
		subscribe("limelight", "botpose", new double[6], event -> changedBotPose(event));
		subscribe("limelight", "json", "", event -> changedJson(event));
	}

	/**
	 * Returns the only {@code AprilTagSubsystem} instantiated in compliance
	 * with the singleton design pattern.
	 * 
	 * @return the only {@code AprilTagSubsystem} instantiated in compliance
	 *         with the singleton design pattern
	 */
	public static AprilTagSubsystem get() {
		return s_subsystem;
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
	TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			m_botpose = new TimestampedDoubleArray(v.getTime(), v.getServerTime(), v.getDoubleArray());
			visionTable.getEntry("Pose2D")
					.setDoubleArray(toPose2DAdvantageScope(m_botpose.value[0], m_botpose.value[1], m_botpose.value[5]));
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
	TimestampedObject<Map<String, double[]>> changedJson(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			var m = toMap(v.getString());
			m_tags = new TimestampedObject<Map<String, double[]>>(v.getTime(), v.getServerTime(), m);
			visionTable.getEntry("Json").setString(toString(m));
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
	static Map<String, double[]> toMap(String s) throws JsonMappingException, JsonProcessingException {
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

}
