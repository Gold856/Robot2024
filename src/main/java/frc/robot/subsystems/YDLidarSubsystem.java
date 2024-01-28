package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YDLidarSubsystem extends SubsystemBase {
	public DoubleArraySubscriber m_lidar = NetworkTableInstance.getDefault()
			.getDoubleArrayTopic("/SmartDashboard/distances").subscribe(new double[360]);
	public double[] m_distances = new double[360];
	private static YDLidarSubsystem s_subsystem;

	/** Creates a new YDLidarSubsystem. */
	public YDLidarSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("YDLidar subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
	}

	public static YDLidarSubsystem get() {
		return s_subsystem;
	}

	@Override
	public void periodic() {
		m_distances = m_lidar.get();
	}

	public double getDistance(int angle) {
		return m_distances[angle];
	}
}
