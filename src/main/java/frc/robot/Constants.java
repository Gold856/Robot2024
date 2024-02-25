// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
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
		// CAN IDs (updated)
		public static final int kCounterWeightPort = 17;
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;
		// Swerve PID values
		public static final double kP = 0.005;
		public static final double kI = 0.045;
		public static final double kD = 0;
		public static final double kIz = 5;

		/*** Distance between center of front wheel and center of back wheel */
		public static final double kWheelBase = 21.5;
		/*** Distance between center of left wheel and center of right wheel */
		public static final double kTrackWidth = 21.5;
		public static final double kSteerPeriod = 0.02;
		// Speed multiplier to make sure the robot doesn't crash into something when
		// testing, because crashing into people's shins would be bad
		public static final double kMaxSpeed = 1;
		public static final double kMinSpeed = 0.1;
		public static final double kTeleopMaxSpeed = 1;
		public static final double kTeleopMaxTurnSpeed = 0.6;
		public static final double kModuleResponseTimeSeconds = 0.02;
		public static final double kGearRatio = 6.12;
		public static final double kWheelDiameter = Units.inchesToMeters(4);

		public static final double kMotorRotationsPerMeter = (1 / kGearRatio) * (Math.PI * kWheelDiameter);

		public static final Translation2d kFrontLeftLocation = new Translation2d(-0.381, 0.381); // -+
		public static final Translation2d kFrontRightLocation = new Translation2d(0.381, 0.381); // ++
		public static final Translation2d kBackLeftLocation = new Translation2d(-0.381, -0.381); // --
		public static final Translation2d kBackRightLocation = new Translation2d(0.381, -0.381); // +-

		public static final int kDriveSmartCurrentLimit = 55;
		public static final int kDrivePeakCurrentLimit = 65;
		public static final int kSteerSmartCurrentLimit = 30;
		public static final int kSteerPeakCurrentLimit = 35;

		// The amount of time to go from 0 to full power in seconds
		public static final double kRampRate = .1;
	}

	public static final class FlywheelConstants {
		public static final int kMasterPort = 50;
		public static final int kFollowerPort = 49;
		public static final boolean kMasterInvert = true;
		public static final boolean kFollowerOppose = false;
		public static final int kSmartCurrentLimit = 50;
		public static final double kPeakCurrentLimit = 60;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kP = 0.000_1;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0.0;
		public static final double kFF = .000_1050;
		public static final double kMaxOutput = 1;

		public static final double kMinOutput = 0;
		public static final double kGearRatio = 2;
		public static final double kAllowedError = 50;
	}

	public static final class ClimbConstants {
		public static final int kLeftPort = 44;
		public static final int kRightPort = 45;
		public static final boolean kLeftInvert = true;
		public static final boolean kRightInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final int kSecondaryCurrentLimit = 70;
		public static final double kMinOutput = -1;
		public static final double kMaxOutput = 1;
		public static final double kP = 0.1;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kTolerance = 1;
		public static final int kMaxExtension = 50;
	}

	public static final class IntakeConstants {
		public static final int kIntakePort = 46;
		public static final int kSmartCurrentLimit = 60;
		public static final int kPeakCurrentLimit = 60;
		public static final boolean kFollowerOppose = false;
		public static final int kGearRatio = 60;
	}

	public static final class IndexerConstants {
		public static final int kIndexerPort = 47;
		public static final int kIndexerPeakCurrentLimit = 55;
		public static final int kIndexerSmartCurrentLimit = 55;
		public static final double kIndexerMaxSpeed = 1;
		public static final double kIndexerMinSpeed = 0.1;

		// Shoot Command Constants
		public static final double kShootTime = 0.5;
		public static final double kShootSpeed = 0.8;
	}

	public static final class PneumaticsConstants {
		public static final int kPneumaticHubID = 50;
		public static final int kLeftAmpBarForwardChannel = 0;
		public static final int kLeftAmpBarReverseChannel = 1;
		public static final int kRightAmpBarForwardChannel = 2;
		public static final int kRightAmpBarReverseChannel = 3;
		public static final int kLeftIntakeForwardChannel = 4;
		public static final int kLeftIntakeReverseChannel = 5;
		public static final int kRightIntakeForwardChannel = 6;
		public static final int kRightIntakeReverseChannel = 7;
		// TODO direction/starting state?
		/** Alias for the solenoid value that makes the intake go down. */
		public static final Value kIntakeDown = Value.kReverse;
		/** Alias for the solenoid value that makes the intake go up. */
		public static final Value kIntakeUp = Value.kForward;
	}
}