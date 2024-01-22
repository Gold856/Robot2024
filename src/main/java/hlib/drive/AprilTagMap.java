package hlib.drive;

import java.io.File;
import java.util.HashMap;
import java.util.Iterator;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * A {@code AprilTagMap} maps the ID of each AprilTag to the 3D transform representing the pose of that AprilTag.
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
	 *            the name of the file
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
	 * Constructs a {@code Pose} from the specified 3D transformation.
	 * 
	 * @param transform
	 *            a {@code double} array representing a 3D transformation
	 * @return a {@code Pose} constructed from the specified 3D transformation
	 */
	public static Pose toPose(double[] transform) {
		double mxx = transform[0];
		double mxy = transform[1];
		double tx = transform[3];
		double ty = transform[7];
		return new Pose(tx, ty, -Math.atan2(mxy, mxx));
	}

}
