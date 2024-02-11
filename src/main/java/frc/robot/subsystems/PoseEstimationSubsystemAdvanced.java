package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Stream;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
	 * A {@code AprilTagMap} maps the ID of each AprilTag to the 3D transform
	 * representing the pose of that AprilTag.
	 * 
	 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
	 * @author Andrew Hwang (u.andrew.h@gmail.com)
	 */
	public static class AprilTagMap extends HashMap<Integer, double[]> {

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
	 * A {@code Map} that maps the ID of each AprilTag to the {@code Pose2d} of that
	 * AprilTag.
	 */
	protected Map<Integer, Pose2d> m_aprilTagPoses = new TreeMap<Integer, Pose2d>();

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 */
	public PoseEstimationSubsystemAdvanced() {
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
	 * Is invoked periodically (every 20 ms approximately).
	 */
	@Override
	public void periodic() {
		super.periodic();
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
