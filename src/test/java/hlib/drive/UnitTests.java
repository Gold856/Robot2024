package hlib.drive;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;

public class UnitTests {

	@Test
	public void testPose() throws Exception {
		assertEquals(0.5 * Math.PI, Pose.normalize(0.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(1.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(-0.5 * Math.PI, Pose.normalize(1.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.0 * Math.PI, Pose.normalize(2.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.5 * Math.PI, Pose.normalize(2.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(3.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(-0.5 * Math.PI, Pose.normalize(3.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.0 * Math.PI, Pose.normalize(4.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.5 * Math.PI, Pose.normalize(4.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(5.0 * Math.PI), 0.01 * Math.PI / 180);

		assertEquals(-0.5 * Math.PI, Pose.normalize(-0.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(-1.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.5 * Math.PI, Pose.normalize(-1.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.0 * Math.PI, Pose.normalize(-2.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(-0.5 * Math.PI, Pose.normalize(-2.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(-3.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.5 * Math.PI, Pose.normalize(-3.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(0.0 * Math.PI, Pose.normalize(-4.0 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(-0.5 * Math.PI, Pose.normalize(-4.5 * Math.PI), 0.01 * Math.PI / 180);
		assertEquals(1.0 * Math.PI, Pose.normalize(-5.0 * Math.PI), 0.01 * Math.PI / 180);
	}

	@Test
	public void testAprilTagMap() throws Exception {
		String deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
				+ "deploy";
		AprilTagMap m = new AprilTagMap(deployPath + File.separator + "2024LimeLightMap.fmap");
		var i = m.entrySet().iterator();
		while (i.hasNext()) {
			var e = i.next();
			var pose = AprilTagMap.toPose(e.getValue());
			System.out.println(String.format("{\"id\": \"%s\", \"pose\": [%.3f, %.3f, %.1f]}%s", e.getKey(), pose.x(),
					pose.y(), pose.yawInDegrees(), i.hasNext() ? "," : ""));
		}
		assertEquals(16, m.size());
	}

	@Test
	public void testPoseEstimationWestCoast() throws Exception {
		var p = new PoseEstimatorWeighted(1.0, 10, 0.1);
		p.update(new Pose(0, 0, 0));
		p.update(new Pose(0.1, 0.1, 0));
		p.add(new PoseCalculatorWestCoastSimple(1.0) {

			double v = 0.0;

			@Override
			public State state() {
				v += 0.1;
				return new State(v, 2 * v, 0.1 * v);
			}
		});
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
	}

	@Test
	public void testPoseEstimationSwerve1() throws Exception {
		var p = new PoseEstimatorWeighted(1.0, 10, 0.1);
		p.update(new Pose(0, 0, 0));
		p.update(new Pose(0.1, 0.1, 0));
		p.add(new PoseCalculatorSwerveSimple(1.0, 1.0) {

			double v = 0.0;

			double[] angles = { 0.1, 0.1, 0.1, 0.1 };

			@Override
			public State state() {
				v += 0.1;
				return new State(angles, new double[] { v, v, v, v }, null);
			}
		});
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
	}

	@Test
	public void testPoseEstimationSwerve2() throws Exception {
		var p = new PoseEstimatorWeighted(1.0, 10, 0.1);
		p.update(new Pose(0, 0, 0));
		p.update(new Pose(0.1, 0.1, 0));
		p.add(new PoseCalculatorSwerveSimple(1.0, 1.0) {

			double v = 0.0;

			double[] angles = { 3 * Math.PI / 4, Math.PI / 4, -3 * Math.PI / 4, -Math.PI / 4 };

			@Override
			public State state() {
				v += 0.1;
				return new State(angles, new double[] { v, v, v, v }, null);
			}
		});
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
		p.periodic();
		System.out.println(p.estimatedPose());
	}

}
