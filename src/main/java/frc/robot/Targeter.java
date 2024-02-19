package frc.robot;

// imports

public interface Targeter {

	public double m_speakerHeight = Constants.ShooterConstants.speakerHeight;
	public double m_shooterLength = Constants.ShooterConstants.shooterLength;
	// aka small delta x and big delta y

	public double getAngle(double distanceMeters);

	public double getRPM(double distanceMeters);

	public double calcActuatorHeightFromAngle(double distanceMeters);

	public double calcActuatorHeightFromDistance(double distanceMeters);

	public class RegressionTargeter { // quadratic regression

		public double getAngle(double distanceMeters) {
			// Calculated using regression from experimentally determined constants
			// Determined with spreadsheet ____
			double angle = 1 * distanceMeters + 1; // TODO get regression math
			return angle;
		}

		public double getRPM(double distanceMeters) {
			// Calculated using regression from experimentally determined constants
			// Determined with spreadsheet ____
			double RPM = 1 * distanceMeters * 1; // TODO get regression math
			return RPM;
		}

		public double calcActuatorHeightFromAngle(double distanceMeters) {
			// get actuator height from getAngle method above
			double angle = getAngle(distanceMeters);
			double actuatorHeight = m_shooterLength / (Math.tan(angle));
			return actuatorHeight;
		}
	}

	public class LookupTargeter {
		// Table values determined by experiment. See _____
		// distanceToTarget [0], bestAngle [1], bestRPM [2]
		public double[][] m_table = { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 } };

		// Utitility function that uses point-slope form of a line to get in between two
		// know points
		// Returns f(x) given [x1, y1], [x2, y2]
		public double pointSlope(double x, double x1, double y1, double x2, double y2) {
			double m = (y2 - y1) / (x2 - x1);
			// point-slope formula
			return m * (x - x1) + y1;
		}

		// Finds the two yaw angles that given angle is in between
		public double[] getPoints(double distanceMeters) {
			// figure out closest distance in table smaller than given distance
			int indexX1 = 0;
			for (int i = 0; i < m_table[0].length; ++i) {
				if (m_table[0][i] <= distanceMeters) {
					indexX1 = i;
				}
			}

			// figure out closest yaw angle in table larger than given angle
			int indexX2 = indexX1 + 1;
			// If first yaw angle is last element in the array, make it the same as the
			// first index
			if (indexX2 >= m_table[0][m_table[0].length - 1]) {
				indexX2 = m_table[0].length - 1;
			}

			double x1 = m_table[0][indexX1];
			double x2 = m_table[0][indexX2];
			double y1Angle = m_table[1][indexX1];
			double y2Angle = m_table[1][indexX2];
			double y1RPM = m_table[2][indexX1];
			double y2RPM = m_table[2][indexX1];
			double[] retPoints = { x1, y1Angle, y1RPM, x2, y2Angle, y2RPM };
			return retPoints;
		}

		public double getAngle(double distanceMeters) {
			double[] points = getPoints(distanceMeters);
			// interpolate angle
			double angle = pointSlope(distanceMeters, points[0], points[1], points[3], points[4]);
			return angle;
		}

		public double getRPM(double distanceMeters) {
			double[] points = getPoints(distanceMeters);
			// interpolate RPM
			double RPM = pointSlope(distanceMeters, points[0], points[2], points[3], points[5]);
			return RPM;
		}

		public double calcActuatorHeightFromAngle(double distanceMeters) {
			// get actuator height from getAngle method above
			double angle = getAngle(distanceMeters);
			double actuatorHeight = m_shooterLength / (Math.tan(angle));
			return actuatorHeight;
		}
	}

	public class PhysicsAndMathTargeter { // using physics and trig

		public double calculateAngle(double distanceMeters) {
			double angle = Math.atan2(m_speakerHeight, distanceMeters);
			return angle;
		}

		public double calculateVelocity(double distanceMeters) {
			double initialVelocity = (distanceMeters * 9.8)
					/ (Math.cos(calculateAngle(distanceMeters)) * (Math.sin(calculateAngle(distanceMeters))));
			return initialVelocity;
		}

		public double calcActuatorHeightFromDistance(double distanceMeters) {
			double actuatorHeight = (m_shooterLength * m_speakerHeight) / distanceMeters;
			return actuatorHeight;
		}
	}
}