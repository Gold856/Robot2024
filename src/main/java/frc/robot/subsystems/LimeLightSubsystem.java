package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The purpose of the {@code LimeLightSubsystem} is to provide the pose of the
 * robot, estimated based on a variety of sources including LimeLight as well as
 * encoders and sensors attached to the robot.
 * 
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 */
public class LimeLightSubsystem extends SubsystemBase {

	/**
	 * A subclass of {@code Pose2d} for simple construction and string
	 * representation of {@code Pose2d} instances.
	 */
	public static class Pose extends Pose2d {

		/**
		 * Constrcuts a {@code Pose}
		 * 
		 * @param x            the x-coordinate value
		 * @param y            the y-coordinate value
		 * @param yawInDegrees the yaw in degrees
		 */
		public Pose(double x, double y, double yawInDegrees) {
			super(x, y, Rotation2d.fromDegrees(yawInDegrees));
		}

		/**
		 * Returns a string representation of this {@code Pose}.
		 */
		@Override
		public String toString() {
			return String.format("[%.1f, %.1f, %.1f degrees]", getX(), getY(), getRotation().getDegrees());
		}

		/**
		 * Returns the average of the specified {@code Pose2d}s.
		 * 
		 * @param poses
		 *              {@code Pose}s
		 * @return the average of the specified {@code Pose2d}s
		 */
		public static Pose2d average(Pose2d... poses) {
			if (poses == null || poses.length == 0)
				return null;
			double x = 0;
			double y = 0;
			double yaw = 0;
			int count = 0;
			for (var pose : poses) {
				if (pose != null) {
					x += pose.getX();
					y += pose.getY();
					yaw += pose.getRotation().getDegrees();
					count++;
				}
			}
			if (count == 0)
				return null;
			return new Pose2d(x / poses.length, y / poses.length, Rotation2d.fromDegrees(yaw / poses.length));
		}

	}

	/**
	 * A {@code Pose2d} representing the center of the field.
	 */
	protected static final Pose2d DEFAULT_POSE = new Pose(0, 0, 0);

	/**
	 * The most recent botpose data obtained from LimeLight.
	 */
	protected double[] m_botpose;

	/**
	 * Constructs a {@code LimeLightSubsystem}.
	 */
	public LimeLightSubsystem() {
		subscribe("limelight", "botpose", new double[6], event -> changedBotPose(event));
	}

	/**
	 * Returns a {@code Pose2d} representing the estimated pose of the robot.
	 * 
	 * @return a {@code Pose2d} representing the estimated pose of the robot
	 *         ({@code null} if it has not been possible to reliably estimate the
	 *         pose of the robot)
	 */
	public Pose2d estimatedPose() {
		if (m_botpose == null)
			return null;
		var pose = new Pose(m_botpose[0], m_botpose[1], m_botpose[5]);
		return pose.equals(DEFAULT_POSE) ? null : pose;
	}

	/**
	 * Adds the specified {@code Listener} to respond to the changes in the
	 * specified topic in the specified table.
	 * 
	 * @param tableName    the name of the table
	 * @param topicName    the name of the topic
	 * @param defaultValue the default value used when no value was obtained
	 *                     regarding the topic
	 * @param listener     a {@code Consumer} responsible for responding to the
	 *                     changes in the specified topic
	 */
	public static void subscribe(String tableName, String topicName, double[] defaultValue,
			Consumer<NetworkTableEvent> listener) {
		NetworkTableInstance i = NetworkTableInstance.getDefault();
		NetworkTable t = i.getTable(tableName);
		var s = t.getDoubleArrayTopic(topicName).subscribe(defaultValue);
		i.addListener(s, EnumSet.of(NetworkTableEvent.Kind.kValueAll), listener);
	}

	/**
	 * Is invoked whenever the "botpose" entry in the "limelight" table changes.
	 * 
	 * @param event a {@code NetworkTableEvent} regarding the change in the
	 *              "botpose" entry in the "limelight" table
	 * @return a {@code double} array from the "botpose" entry in the "limelight"
	 *         table
	 */
	protected double[] changedBotPose(NetworkTableEvent event) {
		try {
			var v = event.valueData.value;
			m_botpose = v.getDoubleArray();
		} catch (Exception e) {
			m_botpose = null;
			// e.printStackTrace();
		}
		return m_botpose;
	}

	/**
	 * Returns the transformation from the estimated pose of of the robot to the
	 * specified target {@code Pose}.
	 * 
	 * @param targetPose the target {@code Pose}
	 * @return the transformation from the estimated pose of of the robot to the
	 *         specified target {@code Pose}; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationTo(Pose2d targetPose) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		return targetPose.minus(pose);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target
	 * position
	 * 
	 * @param targetPosition the target position
	 * @return the transformation needed for the robot to face toward the specified
	 *         target
	 *         position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationToward(Translation2d targetPosition) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		Translation2d diff = targetPosition.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(pose.getTranslation(), diff.getAngle());
		return targetPose.minus(pose);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position and remain the specified distance away fron the target
	 * position.
	 * 
	 * @param targetPosition   the target position whose x and y-coordinate values
	 *                         are in meters
	 * @param distanceToTarget the desired distance in meters to the target
	 * @return the transformation needed for the robot to face toward the specified
	 *         target position and remain the specified distance away fron the
	 *         target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationToward(Translation2d targetPosition, double distanceToTarget) {
		var pose = estimatedPose();
		if (pose == null)
			return null;
		Translation2d diff = targetPosition.minus(pose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(pose.getTranslation().plus(diff.times(1 - distanceToTarget / diff.getNorm())),
				diff.getAngle());
		return targetPose.minus(pose);
	}

}
