package frc.aster;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public final class Constants {

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			/** Left middle button */
			public static final int kSquare = 1;
			/** Bottom button */
			public static final int kX = 2;
			/** Right middle button */
			public static final int kCircle = 3;
			/** Top button */
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kLeftTrigger = 7;
			public static final int kRightTrigger = 8;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {
		public static final int kFrontLeftID = 2;
		public static final boolean kFrontLeftInvert = false;
		public static final int kBackLeftID = 4;
		public static final boolean kBackLeftOppose = false;

		public static final int kFrontRightID = 3;
		public static final boolean kFrontRightInvert = true;
		public static final int kBackRightID = 5;
		public static final boolean kBackRightOppose = false;

		// TODO re-evaluate current limits
		public static final int kSmartCurrentLimit = 55;
		public static final double kPeakCurrentLimit = 65;
		public static final int kPeakCurrentDurationMillis = 0;

		// TODO PIDS
		public static final double kP = .14;// 0.198;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kFineTurningSpeed = .3;

		// navX stuff
		public static final SPI.Port kGyroPort = SPI.Port.kMXP;
		// TODO is gyro truly reversed?
		public static final boolean kGyroReversed = true;

		// TODO CHANGE ADJUST PIDS
		public static final double kTurnP = 0.006; // was 0.005
		public static final double kTurnI = 0; // was 0.003
		public static final double kTurnD = 0; // 0.0
		public static final double kTurnTolerance = 0.5;
		// Horizontal distance between the wheels
		public static final double kTrackwidthMeters = Units.inchesToMeters(20.5);
		public static final double kWheelDiameterMeters = Units.inchesToMeters(5.75);
		public static final double kGearRatio = 9.4;
		public static final double kTurningMultiplier = .45;
		public static final double kRampRate = .1;// 1?
		public static final double kSpeedLimitFactor = 1.0;
		/**
		 * Converts native encoder units(revolutions) to meters
		 * <p>
		 * Native units are in revolutions, 1 / gearRatio gives us how many revolutions
		 * the wheel has turned, and multiplying that by the wheel circumference(pi
		 * times the wheel diameter) gives the distance the robot has moved in meters
		 */
		public static final double kEncoderPositionConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters;
		/**
		 * Converts native encoder units(RPM) to meters per minute
		 * <p>
		 * Native units are in revolutions per minute, 1 / gearRatio gives us how many
		 * revolutions the wheel has turned per minute, and multiplying that by the
		 * wheel circumference(pi times the wheel diameter) gives wheel velocity in
		 * meters per minute, and dividing that by 60 gives wheel velocity in meters per
		 * second. It's in meters per second because that's the unit
		 * DifferentialDriveWheelSpeeds uses
		 */
		public static final double kEncoderVelocityConversionFactor = (1 / DriveConstants.kGearRatio) * Math.PI
				* DriveConstants.kWheelDiameterMeters / 60;
		// Balance Stuff
		public static final double kBalanceP = 0.0065;
		public static final double kBalanceI = 0.000;
		public static final double kBalanceD = 0.001;
		public static final double upStationSpeed = 0.55;
		public static final double kLimelightTurnP = 0.05;
		public static final double kTurnMinSpeed = 0.08;

	}

	public static final class GripperConstants {
		public static final double kGripperOpenPosition = 0.5;// TODO is this correct
		public static final boolean kInvert = false;
		public static final int kGripperID = 9;
		public static final int kSmartCurrentLimit = 15;// TODO evaluate current limit (37 for gripper, 15 for wheel)
		public static final double kCloseTime = 3000; // TODO: change as needed
		public static final double kMovePower = 0.7; // TODO check (0.2 for gripper, 0.4 for wheel gripper)
		public static final double kHoldPower = 0.08; // TODO: change as needed (0.1 for gripper, 0.03 for wheel)
	}

	public static final class LimelightConstants {
		// TODO make real numbers
		public static final double zTolerance = 0.2;
		public static final double xTolerance = 0.2;
		public static final int kRollingMedianSize = 10;
		public static final double kSlowDownDistance = 0.5;
		public static final double kSlowDownDistanceSquared = kSlowDownDistance * kSlowDownDistance;
		public static final double kSpeed = 0.35;
	}

}
