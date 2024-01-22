package frc.robot.subsystems;

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

public class AprilTagSubsystem extends SubsystemBase {

	public static AprilTagSubsystem s_subsystem;

	private TimestampedDoubleArray botpose;

	private TimestampedObject<Map<String, double[]>> tags;

	protected NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

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

	public static AprilTagSubsystem get() { // returns the first instance called, guarantees that there's not
		return s_subsystem;
	}

	public TimestampedDoubleArray botpose() {
		return botpose;
	}

	public TimestampedObject<Map<String, double[]>> tags() {
		return tags;
	}

	public static void subscribe(String tableName, String topicName, String defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance ins = NetworkTableInstance.getDefault();
		NetworkTable table = ins.getTable(tableName);
		var s = table.getStringTopic(topicName).subscribe(defaultValue);
		ins.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	public static void subscribe(String tableName, String topicName, double[] defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance ins = NetworkTableInstance.getDefault();
		NetworkTable table = ins.getTable(tableName);
		var s = table.getDoubleArrayTopic(topicName).subscribe(defaultValue);
		ins.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	/**
	 * A method run periodically (every 20 ms).
	 */
	@Override
	public void periodic() {
	}

	TimestampedDoubleArray changedBotPose(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			botpose = new TimestampedDoubleArray(v.getTime(), v.getServerTime(), v.getDoubleArray());
			visionTable.getEntry("Pose2D")
					.setDoubleArray(toPose2DAdvantageScope(botpose.value[0], botpose.value[1], botpose.value[5]));
			return botpose;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	TimestampedObject<Map<String, double[]>> changedJson(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			var m = toMap(v.getString());
			tags = new TimestampedObject<Map<String, double[]>>(v.getTime(), v.getServerTime(), m);
			visionTable.getEntry("Json").setString(toString(m));
			return tags;
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		}
	}

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

	static double[] toPose2DAdvantageScope(double x, double y, double angleInDegrees) {
		return new double[] { x + 8.27, y + 4.1, angleInDegrees * Math.PI / 180 };
	}

	String toString(Map<String, double[]> m) {
		if (m != null && m.size() > 0) {
			Stream<String> stream = m.entrySet().stream().map(e -> e.getKey() + "=" + Arrays.toString(e.getValue()));
			return stream.reduce((e1, e2) -> e1 + e2).get();
		}
		return "";
	}

}
