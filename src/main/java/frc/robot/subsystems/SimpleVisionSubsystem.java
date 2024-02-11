package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleVisionSubsystem extends SubsystemBase {

	private DoubleArraySubscriber m_subscription;
	private Pose3d m_rawPose;
	private Pose3d m_filteredPose;
	private double m_distance;
	private double m_angle;
	private MedianFilter m_angleMedian;
	private LinearFilter m_angleFilter;

	public SimpleVisionSubsystem() {
		// m_angleMedian = new MedianFilter(10);
		m_angleFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
		setupLimelight();
		setupSubscription();
	}

	@Override
	public void periodic() {
		// get the data from limelight
		updateRawPose();
		// filter out garbage
		updateFilteredPose();
		// do the math right now to convert pose into
		// angle/distance to target
		updateSavedPositions();
	}

	// [Tag]x---- [robot points here]
	// \ | /
	// \ z |
	// \ | /
	// \A|B|
	// \|/
	// / ----- /
	// / robot /
	// --------
	//
	// Limelight gives us distances X and z, and angle B
	// We want to compute A+B as the angle to target
	//
	private void updateSavedPositions() {
		double z = m_filteredPose.getZ();
		double x = m_filteredPose.getX();
		double a = Math.toDegrees(Math.atan2(z, x));
		double b = Math.toDegrees(m_filteredPose.getRotation().getAngle()) + 90;
		SmartDashboard.putNumber("a", a);
		SmartDashboard.putNumber("b", b);
		double angle;
		if (x > 0) {
			angle = -180 - (a - b);
		} else {
			angle = -(a + b);
		}
		if (Math.abs(angle) < 45) {
			m_angle = m_angleFilter.calculate(angle);
			m_distance = z;
		}
		SmartDashboard.putNumber("limelight angle to turn", m_angle);
		SmartDashboard.putNumber("limelight distance", m_distance);
	}

	// When a tag is in frame, returns the distance from the camera to the tag
	public double getDistance() {
		return m_distance;
	}

	// When a tag is in frame, returns:
	// 0 if the tag is directly in front of camera
	// an angle < 0 if the tag is left of camera
	// an angle > 0 if the tag is right of camera
	public double getAngle() {
		return m_angle;
	}

	private void setupLimelight() {
		// for bare-bones, we don't need any special setup
	}

	private void setupSubscription() {
		// subscribe to camerapose-targetspace

		// Subscriber to camerpose_targetspace
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable table = i.getTable("limelight");
		var topic = table.getDoubleArrayTopic("camerapose_targetspace");
		m_subscription = topic.subscribe(new double[6]);
	}

	private void updateRawPose() {
		// pull doubles from table and store them in a Pose3d
		m_rawPose = toPose3D(m_subscription.get());
	}

	private void updateFilteredPose() {
		// TODO: add filtering
		m_filteredPose = m_rawPose;
	}

	// From LimelightHelpers.java open source code
	// https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java#L385
	private static Pose3d toPose3D(double[] inData) {
		if (inData.length < 6) {
			System.err.println("Bad LL 3D Pose Data!");
			return new Pose3d();
		}
		return new Pose3d(
				new Translation3d(inData[0], inData[1], inData[2]),
				new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
						Units.degreesToRadians(inData[5])));
	}

}
