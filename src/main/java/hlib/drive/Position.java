package hlib.drive;

/**
 * A {@code Position} represents a position in a 2-dimensional space.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class Position {

	/**
	 * The x-coordinate value of this {@code Position}.
	 */
	protected double x;

	/**
	 * The y-coordinate value of this {@code Position}.
	 */
	protected double y;

	/**
	 * Constructs a {@code Position}.
	 * 
	 * @param x
	 *            the x-coordinate value of the {@code Position}
	 * @param y
	 *            the y-coordinate value of the {@code Position}
	 */
	public Position(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Returns a {@code String} representation of this {@code Position}.
	 * 
	 * @return a {@code String} representation of this {@code Position}
	 */
	@Override
	public String toString() {
		return String.format("(%.3f, %.3f)", x, y);
	}

	/**
	 * Determines whether or not the given {@code Object} is equal to this {@code Position}.
	 * 
	 * @return {@code true} if the given {@code Object} is equal to this {@code Position}; {@code false} otherwise
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Position) {
			Position p = (Position) o;
			return this.x == p.x && this.y == p.y;
		} else
			return false;
	}

	/**
	 * Returns the x-coordinate value of this {@code Position}.
	 * 
	 * @return the x-coordinate value of this {@code Position}
	 */
	public double x() {
		return x;
	}

	/**
	 * Returns the y-coordinate value of this {@code Position}.
	 * 
	 * @return the y-coordinate value of this {@code Position}
	 */
	public double y() {
		return y;
	}

	/**
	 * Returns the angle in radians from the origin to the this {@code Position} measured from the positive x-axis.
	 * 
	 * @return the angle in radians from the origin to the this {@code Position} measured from the positive x-axis
	 */
	public double angleInRadians() {
		return Math.atan2(y, x);
	}

	/**
	 * Returns the angle in degrees from the origin to the this {@code Position} measured from the positive x-axis.
	 * 
	 * @return the angle in degrees from the origin to the this {@code Position} measured from the positive x-axis
	 */
	public double angleInDegrees() {
		return Math.toDegrees(angleInRadians());
	}

	/**
	 * Returns the displacement from this {@code Position} to the specified {@code Position}.
	 * 
	 * @param p
	 *            a {@code Position} to which the displacement is calculated
	 * @return the displacement from this {@code Position} to the specified {@code Position}
	 */
	public Position displacementTo(Position p) {
		return new Position(p.x - x, p.y - y);
	}

	/**
	 * Returns the distance between this {@code Position} and the specified {@code Position}.
	 * 
	 * @param p
	 *            a {@code Position}
	 * @return the distance between this {@code Position} and the specified {@code Position}
	 */
	public double distance(Position p) {
		double x = this.x - p.x;
		double y = this.y - p.y;
		return Math.sqrt(x * x + y * y);
	}

	/**
	 * Returns the {@code Position} resulting from rotating this {@code Position} around the origin by the specified
	 * angle in radians.
	 * 
	 * @param angleInRadians
	 *            an angle in radians
	 * @return the {@code Position} resulting from rotating this {@code Position} around the origin by the specified
	 *         angle in radians
	 */
	public Position rotate(double angleInRadians) {
		return new Position(Math.cos(angleInRadians) * x - Math.sin(angleInRadians) * y,
				Math.sin(angleInRadians) * x + Math.cos(angleInRadians) * y);
	}

	/**
	 * Returns the {@code Position} resulting from translating this {@code Position} by the coordinates of the specified
	 * {@code Position}.
	 * 
	 * @param p
	 *            a {@code Position}
	 * @return the {@code Position} resulting from translating this {@code Position} by the coordinates of the specified
	 *         {@code Position}
	 */
	public Position translate(Position p) {
		return new Position(x + p.x, y + p.y);
	}

}
