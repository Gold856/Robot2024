package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleVisionSubsystem extends SubsystemBase {
	private final DoubleSubscriber m_subscription;
	private double m_angle;

	public SimpleVisionSubsystem() {
		// Setup subscription
		m_subscription = NetworkTableInstance.getDefault().getTable("limelight")
				.getDoubleTopic("tx")
				.subscribe(0);
	}

	@Override
	public void periodic() {
		// get the data from limelight
		m_angle = m_subscription.get();
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
