// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
	private Rotation2d m_heading = new Rotation2d();
	private final SysIdRoutine m_sysidRoutine;
	private final PIDController m_xController = new PIDController(0, 0, 0);
	private final PIDController m_yController = new PIDController(0, 0, 0);
	private final PIDController m_headingController = new PIDController(0, 0, 0);

	private final ProtobufPublisher<Pose2d> m_posePublisher;
	private final StructPublisher<ChassisSpeeds> m_targetChassisSpeedsPublisher;
	private final StructArrayPublisher<SwerveModuleState> m_targetModuleStatePublisher;
	private final StructArrayPublisher<SwerveModulePosition> m_targetModulePositionPublisher;
	private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatePublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		m_posePublisher = NetworkTableInstance.getDefault().getProtobufTopic("/SmartDashboard/Pose", Pose2d.proto)
				.publish();
		m_targetChassisSpeedsPublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Target Chassis Speeds", ChassisSpeeds.struct).publish();
		m_targetModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Target Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_targetModulePositionPublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Swerve Module Positions", SwerveModulePosition.struct)
				.publish();
		m_currentModuleStatePublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("/SmartDashboard/Current Swerve Modules States", SwerveModuleState.struct)
				.publish();
		m_frontLeft = new SwerveModule(kFrontLeftCANCoderPort, kFrontLeftDrivePort, kFrontLeftSteerPort, kvFrontLeft,
				kaFrontLeft);
		m_frontRight = new SwerveModule(kFrontRightCANCoderPort, kFrontRightDrivePort, kFrontRightSteerPort,
				kvFrontRight, kaFrontRight);
		m_backLeft = new SwerveModule(kBackLeftCANCoderPort, kBackLeftDrivePort, kBackLeftSteerPort, kvBackLeft,
				kaBackLeft);
		m_backRight = new SwerveModule(kBackRightCANCoderPort, kBackRightDrivePort, kBackRightSteerPort, kvBackRight,
				kaBackRight);
		var config = new SysIdRoutine.Config(Units.Volts.of(2.5).per(Units.Seconds.of(1)), null, Units.Seconds.of(3));
		m_sysidRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism(volt -> {
			var state = new SwerveModuleState(volt.magnitude(), new Rotation2d(Math.PI / 2));
			m_frontLeft.setModuleState(state);
			m_frontRight.setModuleState(state);
			m_backLeft.setModuleState(state);
			m_backRight.setModuleState(state);
		}, null, this));
		m_gyro.zeroYaw();
		resetEncoders();
		// Wait 100 milliseconds to let all the encoders reset
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading(), getModulePositions());
	}

	/**
	 * Gets the robot's heading from the gyro.
	 * 
	 * @return The heading
	 */
	public Rotation2d getHeading() {
		return m_gyro.getRotation2d();
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
	 * Returns robot pose.
	 * 
	 * @return The pose of the robot.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
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

	public SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds, boolean isFieldRelative,
			boolean isOpenLoop) {
		if (isFieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
		if (isOpenLoop) {
			SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxVoltage);
		} else {
			SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxDrivetrainVelocity);
		}
		// Get the current module angles
		double[] moduleAngles = { m_frontLeft.getModuleAngle(), m_frontRight.getModuleAngle(),
				m_backLeft.getModuleAngle(), m_backRight.getModuleAngle() };
		for (int i = 0; i < states.length; i++) {
			// Optimize target module states
			states[i] = SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(moduleAngles[i]));
		}
		return states;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speeds          The chassis speeds.
	 * @param isFieldRelative Whether or not the chassis speeds are field-relative.
	 * @param isOpenLoop      Whether or not the modules will use open-loop control
	 *                        (no PID on drive motors).
	 */
	public void setModuleStates(SwerveModuleState[] states) {
		m_targetModuleStatePublisher.set(states);
		m_frontLeft.setModuleState(states[0]);
		m_frontRight.setModuleState(states[1]);
		m_backLeft.setModuleState(states[2]);
		m_backRight.setModuleState(states[3]);
	}

	/**
	 * Drives the robot.
	 * 
	 * @param speedFwd        The forward speed in voltage
	 * @param speedSide       The sideways speed in voltage
	 * @param speedRot        The rotation speed in voltage
	 * @param isFieldRelative Whether or not the speeds are relative to the field
	 */
	public void drive(double speedFwd, double speedSide, double speedRot, boolean isFieldRelative) {
		setModuleStates(calculateModuleStates(new ChassisSpeeds(speedFwd, speedSide, speedRot), isFieldRelative, true));
	}

	public void followTrajectory(Pose2d pose, SwerveSample sample) {
		var vx = m_xController.calculate(pose.getX(), sample.x) + sample.vx;
		var vy = m_yController.calculate(pose.getY(), sample.y) + sample.vy;
		var omega = m_headingController.calculate(pose.getRotation().getRadians(), sample.heading) + sample.omega;
		var speeds = new ChassisSpeeds(vx, vy, omega);
		var moduleSpeeds = calculateModuleStates(speeds, true, false);
		var moduleAccels = calculateModuleStates(new ChassisSpeeds(sample.ax, sample.ay, sample.alpha), true, false);
		var states = new SwerveModuleState[4];
		m_targetChassisSpeedsPublisher.set(speeds);
		states[0] = m_frontLeft.setModuleStateClosedLoop(moduleSpeeds[0], moduleAccels[0].speedMetersPerSecond);
		states[1] = m_frontRight.setModuleStateClosedLoop(moduleSpeeds[1], moduleAccels[1].speedMetersPerSecond);
		states[2] = m_backLeft.setModuleStateClosedLoop(moduleSpeeds[2], moduleAccels[2].speedMetersPerSecond);
		states[3] = m_backRight.setModuleStateClosedLoop(moduleSpeeds[3], moduleAccels[3].speedMetersPerSecond);
		m_targetModuleStatePublisher.set(states);
		m_heading = new Rotation2d(omega * 0.02).plus(m_heading);
	}

	public AutoFactory autoFactory() {
		return Choreo.createAutoFactory(this, this::getPose, this::followTrajectory,
				() -> false, new AutoBindings());
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("Current Position",
		// getModulePositions()[0].distanceMeters);
		SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());
		SmartDashboard.putNumber("Heading Radians", getHeading().getRadians());
		m_posePublisher.set(m_odometry.update(getHeading(), getModulePositions()));
		m_targetModulePositionPublisher.set(getModulePositions());
		SwerveModuleState[] states = { m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
				m_backLeft.getModuleState(), m_backRight.getModuleState() };
		m_currentModuleStatePublisher.set(states);
	}

	/**
	 * Creates a command to drive the robot with joystick input.
	 *
	 * @param forwardSpeed    Forward speed supplier. Positive values make the robot
	 *                        go forward (+X direction).
	 * @param strafeSpeed     Strafe speed supplier. Positive values make the robot
	 *                        go to the left (+Y direction).
	 * @param rotation        Rotation speed supplier. Positive values make the
	 *                        robot rotate CCW.
	 * @param isFieldRelative Supplier for determining if driving should be field
	 *                        relative.
	 * @return A command to drive the robot.
	 */
	public Command driveCommand(Supplier<Double> forwardSpeed, Supplier<Double> strafeSpeed,
			Supplier<Double> rotation, BooleanSupplier isFieldRelative) {
		return run(() -> {
			// Get the forward, strafe, and rotation speed, using a deadband on the joystick
			// input so slight movements don't move the robot
			double rotSpeed = MathUtil.applyDeadband(rotation.get(), ControllerConstants.kDeadzone);
			rotSpeed = -Math.signum(rotSpeed) * Math.pow(rotSpeed, 2) * kTeleopMaxTurnVoltage;

			double fwdSpeed = MathUtil.applyDeadband(forwardSpeed.get(), ControllerConstants.kDeadzone);
			fwdSpeed = -Math.signum(fwdSpeed) * Math.pow(fwdSpeed, 2) * kTeleopMaxVoltage;

			double strSpeed = MathUtil.applyDeadband(strafeSpeed.get(), ControllerConstants.kDeadzone);
			strSpeed = -Math.signum(strSpeed) * Math.pow(strSpeed, 2) * kTeleopMaxVoltage;

			drive(fwdSpeed, strSpeed, rotSpeed, isFieldRelative.getAsBoolean());
		}).withName("DefaultDriveCommand");
	}

	/**
	 * Creates a command to reset the gyro heading to zero.
	 * 
	 * @return A command to reset the gyro heading.
	 */
	public Command resetHeadingCommand() {
		return runOnce(m_gyro::zeroYaw);
	}

	public Command resetOdometryCommand(Pose2d pose) {
		return runOnce(() -> m_odometry.resetPosition(getHeading(), getModulePositions(), pose));
	}

	/**
	 * Creates a command to run a SysId quasistatic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.quasistatic(direction);
	}

	/**
	 * Creates a command to run a SysId dynamic test.
	 * 
	 * @param direction The direction to run the test in.
	 * @return The command.
	 */
	public Command sysidDynamic(SysIdRoutine.Direction direction) {
		return m_sysidRoutine.dynamic(direction);
	}
}
