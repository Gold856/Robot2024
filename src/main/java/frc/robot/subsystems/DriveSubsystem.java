// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
	private final SwerveDriveOdometry m_odometry;
	private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
	private Rotation2d m_heading = new Rotation2d(Math.PI / 2);
	private final Field2d m_field = new Field2d();

	private final ProtobufPublisher<Pose2d> m_posePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
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
					kFrontLeftDriveInverted);

			m_frontRight = new SwerveModule(
					kFrontRightCANCoderPort,
					kFrontRightDrivePort,
					kFrontRightSteerPort,
					kFrontRightDriveInverted);

			m_backLeft = new SwerveModule(
					kBackLeftCANCoderPort,
					kBackLeftDrivePort,
					kBackLeftSteerPort,
					kBackLeftDriveInverted);

			m_backRight = new SwerveModule(
					kBackRightCANCoderPort,
					kBackRightDrivePort,
					kBackRightSteerPort,
					kBackRightDriveInverted);
		}
		m_gyro.zeroYaw();
		resetEncoders();
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
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
		m_frontLeft.resetDriveEncoder();
		m_frontRight.resetDriveEncoder();
		m_backLeft.resetDriveEncoder();
		m_backRight.resetDriveEncoder();
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

		// m_pose = m_pose.plus(transform);
		// m_heading = m_pose.getRotation();
		// m_posePublisher.set(m_pose);
		SmartDashboard.putNumber("Heading", getHeading().getRadians());

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
		m_targetModuleStatePublisher.set(states);
		m_field.setRobotPose(m_pose);
		return states;
	}

	/**
	 * Stops all the motors.
	 */
	public void stopDriving() {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(0, 0, 0), true));
	}

	/**
	 * Gets the module positions for each swerve module.
	 * 
	 * @return The module positions, in order of FL, FR, BL, BR
	 */
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] { m_frontLeft.getModulePosition(), m_frontRight.getModulePosition(),
				m_backLeft.getModulePosition(), m_backRight.getModulePosition() };
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param moduleStates The module states, in order of FL, FR, BL, BR
	 */
	public void setModuleStates(SwerveModuleState[] moduleStates) {
		m_frontLeft.setModuleState(moduleStates[0]);
		m_frontRight.setModuleState(moduleStates[1]);
		m_backLeft.setModuleState(moduleStates[2]);
		m_backRight.setModuleState(moduleStates[3]);
	}

	/**
	 * Directly sets the module angle. Do not use for general driving.
	 * 
	 * @param angle The angle in degrees
	 */
	public void setModuleAngles(double angle) {
		m_frontLeft.setAngle(angle);
		m_frontRight.setAngle(angle);
		m_backLeft.setAngle(angle);
		m_backRight.setAngle(angle);
	}

	/**
	 * Sets module states for each swerve module.
	 * 
	 * @param speedFwd        The forward speed
	 * @param speedSide       The sideways speed
	 * @param speedRot        The rotation speed
	 * @param isFieldRelative Whether or not the speeds are relative to the field
	 */
	public void setModuleStates(double speedFwd, double speedSide, double speedRot, boolean isFieldRelative) {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(speedFwd, speedSide, speedRot), isFieldRelative));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Current Position", getModulePositions()[0].distanceMeters);
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
	}

	/**
	 * Creates a command to drive the robot with joystick input.
	 * 
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotationAxis) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double fwdSpeed = -MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			double strSpeed = -MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			double rotSpeed = -MathUtil.applyDeadband(rotationAxis.get(), ControllerConstants.kDeadzone);

			setModuleStates(calculateModuleStates(new ChassisSpeeds(fwdSpeed, strSpeed, rotSpeed), true));
		});
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeadingCommand() {
		return runOnce(m_gyro::zeroYaw);
	}

	/**
	 * Creates a command to reset the drive encoders to zero.
	 * 
	 * @return A command to reset the drive encoders.
	 */
	public Command resetEncodersCommand() {
		return runOnce(() -> {
			resetEncoders();
			m_odometry.resetPosition(getHeading(), getModulePositions(), new Pose2d());
		});
	}

	/**
	 * Creates a command to align the swerve modules to zero degrees relative to the
	 * robot.
	 * 
	 * @return A command to align the swerve modules.
	 */
	public Command alignModulesToZeroComamnd() {
		return run(() -> {
			m_kinematics.resetHeadings(
					new Rotation2d[] { new Rotation2d(0), new Rotation2d(0), new Rotation2d(0), new Rotation2d(0) });
			setModuleStates(0, 0, 0, false);
		}).raceWith(Commands.waitSeconds(5));
	}
}
