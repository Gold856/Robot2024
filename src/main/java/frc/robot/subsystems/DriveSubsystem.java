// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	private SwerveModule m_frontLeft;
	private SwerveModule m_frontRight;
	private SwerveModule m_backLeft;
	private SwerveModule m_backRight;

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private static DriveSubsystem s_subsystem;
	private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
	private Rotation2d m_heading = new Rotation2d(Math.PI / 2);
	private ProtobufPublisher<Pose2d> m_posePublisher;
	private final Field2d m_field = new Field2d();

	private StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Motor subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

		SmartDashboard.putData("Field", m_field);
		m_posePublisher = NetworkTableInstance.getDefault().getProtobufTopic("/SmartDashboard/Pose", Pose2d.proto)
				.publish();
		m_targetModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_currentModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
				.publish();
		// Initialize modules
		{
			m_frontLeft = new SwerveModule(
					kFrontLeftCANCoderPort,
					kFrontLeftDrivePort,
					kFrontLeftSteerPort,
					kFrontLeftEncoderOffset,
					kFrontLeftDriveInverted);

			m_frontRight = new SwerveModule(
					kFrontRightCANCoderPort,
					kFrontRightDrivePort,
					kFrontRightSteerPort,
					kFrontRightEncoderOffset,
					kFrontRightDriveInverted);

			m_backLeft = new SwerveModule(
					kBackLeftCANCoderPort,
					kBackLeftDrivePort,
					kBackLeftSteerPort,
					kBackLeftEncoderOffset,
					kBackLeftDriveInverted);

			m_backRight = new SwerveModule(
					kBackRightCANCoderPort,
					kBackRightDrivePort,
					kBackRightSteerPort,
					kBackRightEncoderOffset,
					kBackRightDriveInverted);
		}
		m_gyro.reset();
		m_gyro.zeroYaw();
		resetEncoders();
	}

	public static DriveSubsystem get() {
		return s_subsystem;
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-m_gyro.getYaw());
	}

	/**
	 * Resets gyro heading to zero.
	 */
	public void resetHeading() {
		m_gyro.reset();
	}

	/**
	 * Resets drive encoders to zero.
	 */
	public void resetEncoders() {
		// Zero drive encoders
		m_frontLeft.getDriveEncoder().setPosition(0);
		m_frontRight.getDriveEncoder().setPosition(0);
		m_backLeft.getDriveEncoder().setPosition(0);
		m_backRight.getDriveEncoder().setPosition(0);
	}

	/**
	 * Sets all modules to zero degrees.
	 */
	public void setWheelRotationToZeroDegrees() {
		setSteerMotors(0, 0, 0, 0);
	}

	/**
	 * Calculates the modules states needed for the robot to achieve the target
	 * chassis speed.
	 * 
	 * @param speeds          The target chassis speed
	 * @param isFieldRelative Whether or not the chassis speeds are field-relative
	 * @return The module states, in order of FL, FR, BL, BR
	 */
	public SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isFieldRelative) {
		if (isFieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}
		var transform = new Transform2d(speeds.vxMetersPerSecond * kModuleResponseTimeSeconds,
				speeds.vyMetersPerSecond * kModuleResponseTimeSeconds, new Rotation2d(
						speeds.omegaRadiansPerSecond * kModuleResponseTimeSeconds));

		m_pose = m_pose.plus(transform);
		m_heading = m_pose.getRotation();
		m_posePublisher.set(m_pose);
		SmartDashboard.putNumber("Heading", getHeading().getRadians());

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
		m_targetModuleStatePublisher.set(states);
		m_field.setRobotPose(m_pose);
		return states;
	}

	/**
	 * Sets the target angles in degrees for each wheel on the robot
	 * 
	 * @param frontLeftAngle  The target angle of the front left wheel in degrees
	 * @param frontRightAngle The target angle of the front right wheel in degrees
	 * @param backLeftAngle   The target angle of the back left wheel in degrees
	 * @param backRightAngle  The target angle of the back right wheel in degrees
	 */
	public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
			double backRightAngle) {
		m_frontLeft.getPIDController().setSetpoint(frontLeftAngle);
		m_frontRight.getPIDController().setSetpoint(frontRightAngle);
		m_backLeft.getPIDController().setSetpoint(backLeftAngle);
		m_backRight.getPIDController().setSetpoint(backRightAngle);
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param moduleStates The module states, in order of FL, FR, BL, BR
	 */
	public void setSwerveStates(SwerveModuleState[] moduleStates) {
		m_frontLeft.setModuleState(moduleStates[0]);
		m_frontRight.setModuleState(moduleStates[1]);
		m_backLeft.setModuleState(moduleStates[2]);
		m_backRight.setModuleState(moduleStates[3]);
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = {
				new SwerveModuleState(m_frontLeft.getDriveSpeed(),
						Rotation2d.fromDegrees(m_frontLeft.getModuleAngle())),
				new SwerveModuleState(m_frontRight.getDriveSpeed(),
						Rotation2d.fromDegrees(m_frontRight.getModuleAngle())),
				new SwerveModuleState(m_backLeft.getDriveSpeed(), Rotation2d.fromDegrees(m_backLeft.getModuleAngle())),
				new SwerveModuleState(m_backRight.getDriveSpeed(),
						Rotation2d.fromDegrees(m_backRight.getModuleAngle())) };
		m_currentModuleStatePublisher.set(states);
	}
}
