package frc.robot.subsystems;

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

	public SimpleVisionSubsystem() {
		setupLimelight();
		setupSubscription();
	}

	@Override
	public void periodic() {
		updateRawPose();
		updateFilteredPose();
	}

	// When a tag is in frame, returns the distance from the camera to the tag
	double getDistance() {
		// return something from filtered pose
		return m_filteredPose.getZ();
	}

	// When a tag is in frame, returns:
	// 0 if the tag is directly in front of camera
	// an angle < 0 if the tag is left of camera
	// an angle > 0 if the tag is right of camera
	double getAngle() {
		// for now,
		// cheat and pretend that displacement is actually an angle
		return -1 * m_filteredPose.getX();
	}

	private void updateFilteredPose() {
		// TODO: add filtering
		m_filteredPose = m_rawPose;
		SmartDashboard.putNumber("visionAngle", getAngle());
		SmartDashboard.putNumber("visionDistance", getDistance());
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
		// pull doubles from table
		m_rawPose = toPose3D(m_subscription.get());
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
