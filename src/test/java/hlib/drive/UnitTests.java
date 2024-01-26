package hlib.drive;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;

public class UnitTests {

	String deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
			+ "deploy";

	@Test
	public void testPose() throws Exception {
		assertEquals(0.5 * Math.PI, Pose.normalize(0.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(1.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(-0.5 * Math.PI, Pose.normalize(1.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.0 * Math.PI, Pose.normalize(2.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.5 * Math.PI, Pose.normalize(2.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(3.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(-0.5 * Math.PI, Pose.normalize(3.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.0 * Math.PI, Pose.normalize(4.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.5 * Math.PI, Pose.normalize(4.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(5.0 * Math.PI), 0.001 * Math.PI);

		assertEquals(-0.5 * Math.PI, Pose.normalize(-0.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(-1.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.5 * Math.PI, Pose.normalize(-1.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.0 * Math.PI, Pose.normalize(-2.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(-0.5 * Math.PI, Pose.normalize(-2.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(-3.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.5 * Math.PI, Pose.normalize(-3.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(0.0 * Math.PI, Pose.normalize(-4.0 * Math.PI), 0.001 * Math.PI);
		assertEquals(-0.5 * Math.PI, Pose.normalize(-4.5 * Math.PI), 0.001 * Math.PI);
		assertEquals(1.0 * Math.PI, Pose.normalize(-5.0 * Math.PI), 0.001 * Math.PI);
	}

	@Test
	public void testAprilTagMap() throws Exception {
		AprilTagMap m = new AprilTagMap(deployPath + File.separator + "2024LimeLightMap.fmap");
		var i = m.entrySet().iterator();
		while (i.hasNext()) {
			var e = i.next();
			var pose = AprilTagMap.toPose(e.getValue());
			System.out.println(String.format("{\"id\": \"%s\", \"pose\": [%.3f, %.3f, %.1f]}%s", e.getKey(), pose.x(),
					pose.y(), pose.yawInDegrees(), i.hasNext() ? "," : ""));
		}
		System.out.println();
		assertEquals(16, m.size());
	}

	@Test
	public void testPoseMove() throws Exception {
		assertEquals("(1.000, 0.000, 135.0 degrees)",
				new Pose(1, 0, Math.PI / 2).move(new Pose(0, 0, 0), new Pose(0, 0, Math.PI / 4)).toString());
		assertEquals("(1.000, 0.000, 135.0 degrees)",
				new Pose(1, 0, Math.PI / 2).move(new Pose(1, 1, 0), new Pose(1, 1, Math.PI / 4)).toString());
		assertEquals("(1.000, 0.000, 135.0 degrees)",
				new Pose(1, 0, Math.PI / 2).move(new Pose(1, 1, Math.PI / 4), new Pose(1, 1, Math.PI / 2)).toString());
		assertEquals("(1.000, 1.414, 135.0 degrees)",
				new Pose(1, 0, Math.PI / 2).move(new Pose(1, 1, Math.PI / 4), new Pose(2, 2, Math.PI / 2)).toString());
		assertEquals("(0.293, 3.121, 135.0 degrees)",
				new Pose(1, 1, Math.PI / 2).move(new Pose(1, 1, Math.PI / 4), new Pose(2, 3, Math.PI / 2)).toString());
		assertEquals("(2.000, -2.000, 0.0 degrees)", new Pose(1, -1, -Math.PI / 4)
				.move(new Pose(1, 1, Math.PI / 4), new Pose(2, 2, Math.PI / 2)).toString());
	}

}
