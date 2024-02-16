package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleVisionSubsystem extends SubsystemBase {
	private final DoubleSubscriber m_subscription;
	private final MedianFilter m_angleFilter;
	private double m_angle;

	public SimpleVisionSubsystem() {
		m_angleFilter = new MedianFilter(10);
		// Setup subscription
		m_subscription = NetworkTableInstance.getDefault().getTable("limelight")
				.getDoubleTopic("tx")
				.subscribe(0);
	}

	@Override
	public void periodic() {
		// get the data from limelight
		var angle = m_subscription.get();
		// ignore unrealistic answers
		m_angle = m_angleFilter.calculate(angle);
		SmartDashboard.putNumber("limelight angle to turn", m_angle);
	}

	/**
	 * Returns the angle of the camera to the tag. Negative if the tag is left of
	 * the camera, positive if the tag is right of the camera.
	 * 
	 * @return The angle of the camera
	 */
	public double getAngle() {
		return m_angle;
	}
}
