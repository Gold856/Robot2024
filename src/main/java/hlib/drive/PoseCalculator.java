package hlib.drive;

/**
 * A {@code PoseCalculator} calculates the pose of an object based on the pose of that object at an earlier time and some
 * changes in that object observed via some sources such as a gyroscope, encoders, etc.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface PoseCalculator {

	/**
	 * Calculates the current pose of an object based on the pose of that object at an earlier time and some changes in
	 * that object observed via some sources such as a gyroscope, encoders, etc.
	 * 
	 * @param previous
	 *            a {@code Pose} representing the pose of the object at an earlier time
	 * @return a {@code Pose} representing the pose calculated by this {@code PoseCalculator}
	 */
	public Pose pose(Pose previous);

}
