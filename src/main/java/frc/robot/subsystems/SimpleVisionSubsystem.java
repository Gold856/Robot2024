package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleVisionSubsystem extends SubsystemBase {
	private final DoubleArraySubscriber m_subscription;
	private double m_distance;
	private double m_angle;
	private MedianFilter m_angleFilter;

	public SimpleVisionSubsystem() {
		m_angleFilter = new MedianFilter(10);
		// Setup subscription
		m_subscription = NetworkTableInstance.getDefault().getTable("limelight")
				.getDoubleArrayTopic("targetpose_robotspace")
				.subscribe(new double[6]);
	}

	@Override
	public void periodic() {
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
	// Limelight gives us distances X and z, and angle B
	// We want to compute A+B as the angle to target
	//
	private void updateSavedPositions() {
		// get the data from limelight
		var pose = m_subscription.get();
		double x = pose[0];
		double z = pose[2];
		double angle = pose[5];
		// ignore unrealistic answers
		if (Math.abs(angle) < 45) {
			m_angle = m_angleFilter.calculate(angle);
			m_distance = x;
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
}
