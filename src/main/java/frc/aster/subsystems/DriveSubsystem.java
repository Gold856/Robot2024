package frc.aster.subsystems;

import java.util.function.Function;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.hal.ControlWord;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.aster.Constants;
import frc.aster.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
	private static DriveSubsystem s_subsystem;

	private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftID, MotorType.kBrushless);
	private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightID, MotorType.kBrushless);
	private final CANSparkMax m_backLeft = new CANSparkMax(DriveConstants.kBackLeftID, MotorType.kBrushless);
	private final CANSparkMax m_backRight = new CANSparkMax(DriveConstants.kBackRightID, MotorType.kBrushless);

	private final RelativeEncoder m_leftEncoder = m_frontLeft.getEncoder();
	private final RelativeEncoder m_rightEncoder = m_frontRight.getEncoder();
	private final SparkPIDController m_leftPIDController = m_frontLeft.getPIDController();
	private final SparkPIDController m_rightPIDController = m_frontRight.getPIDController();

	private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);

	private final DifferentialDriveOdometry m_odometry;

	ControlWord m_controlWord = new ControlWord();
	/** Prevents the motors from continuously being set to brake or coast mode */
	private boolean wasDisabled;

	/**
	 * A {@code State} represents the state of this {@code DriveSubsystem} at a
	 * certain moment.
	 */
	record State(double leftEncoderPosition, double rightEncoderPosition, double yawInDegrees) {

		public State(double leftEncoderPosition, double rightEncoderPosition, double yawInDegrees) {
			this.leftEncoderPosition = leftEncoderPosition;
			this.rightEncoderPosition = rightEncoderPosition;
			this.yawInDegrees = yawInDegrees > 180 ? yawInDegrees - 360
					: yawInDegrees <= -180 ? yawInDegrees + 360 : yawInDegrees;
		}
	};

	/**
	 * The current {@code State} of this {@code DriveSubsystem}.
	 */
	State m_state;

	public DriveSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Drive subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
		m_frontLeft.restoreFactoryDefaults();
		m_frontLeft.setInverted(DriveConstants.kFrontLeftInvert);
		m_frontLeft.setIdleMode(IdleMode.kBrake);
		m_frontLeft.enableVoltageCompensation(12);
		m_frontLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
		m_frontLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
				DriveConstants.kPeakCurrentDurationMillis);
		m_frontLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

		m_backLeft.restoreFactoryDefaults();
		m_backLeft.setIdleMode(IdleMode.kCoast);
		m_backLeft.enableVoltageCompensation(12);
		m_backLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
		m_backLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
				DriveConstants.kPeakCurrentDurationMillis);
		m_backLeft.setOpenLoopRampRate(DriveConstants.kRampRate);
		m_backLeft.follow(m_frontLeft, DriveConstants.kBackLeftOppose);

		m_frontRight.restoreFactoryDefaults();
		m_frontRight.setInverted(DriveConstants.kFrontRightInvert);
		m_frontRight.setIdleMode(IdleMode.kBrake);
		m_frontRight.enableVoltageCompensation(12);
		m_frontRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
		m_frontRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
				DriveConstants.kPeakCurrentDurationMillis);
		m_frontRight.setOpenLoopRampRate(DriveConstants.kRampRate);

		m_backRight.restoreFactoryDefaults();
		m_backRight.setIdleMode(IdleMode.kCoast);
		m_backRight.enableVoltageCompensation(12);
		m_backRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
		m_backRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
				DriveConstants.kPeakCurrentDurationMillis);
		m_backRight.setOpenLoopRampRate(DriveConstants.kRampRate);
		m_backRight.follow(m_frontRight, DriveConstants.kBackRightOppose);

		m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderPositionConversionFactor);
		m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

		m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderPositionConversionFactor);
		m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

		m_leftPIDController.setP(DriveConstants.kP);
		m_leftPIDController.setI(DriveConstants.kI);
		m_leftPIDController.setIZone(DriveConstants.kIz);
		m_leftPIDController.setD(DriveConstants.kD);
		m_leftPIDController.setFF(DriveConstants.kFF);
		m_leftPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
		m_leftPIDController.setFeedbackDevice(m_leftEncoder);

		m_rightPIDController.setP(DriveConstants.kP);
		m_rightPIDController.setI(DriveConstants.kI);
		m_rightPIDController.setIZone(DriveConstants.kIz);
		m_rightPIDController.setD(DriveConstants.kD);
		m_rightPIDController.setFF(DriveConstants.kFF);
		m_rightPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
		m_rightPIDController.setFeedbackDevice(m_rightEncoder);

		m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
		m_leftEncoder.setPosition(0);
		m_rightEncoder.setPosition(0);
		m_state = RobotBase.isSimulation() ? new State(0, 0, 0)
				: new State(getLeftEncoderPosition(), getRightEncoderPosition(), getHeading());
	}

	@Override
	public void close() {
		m_frontLeft.close();
		m_frontRight.close();
		m_backLeft.close();
		m_backRight.close();
	}

	public static DriveSubsystem get() {
		return s_subsystem;
	}

	public void periodic() {
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
		// wasDisabled exists so the motors aren't constantly set to brake or coast mode
		// Without it, the code would continuously set the motors in brake or coast mode
		// If the robot is enabled, put the motors in brake mode
		if (isEnabledFast() && wasDisabled) {
			setFrontBrake();
			setBackCoast();
			wasDisabled = false;
			// If the robot is disabled, put the motors in coast mode
		} else if (!isEnabledFast() && !wasDisabled) {
			// setFrontCoast();
			setBackBrake();
			wasDisabled = true;
		}
		SmartDashboard.putString("DriveSubsystem", "" + m_state);
		SmartDashboard.putString("DifferentialDriveOdometry", "" + m_odometry.getPoseMeters());
	}

	/**
	 * @return The left encoder position (meters)
	 */
	public double getLeftEncoderPosition() {
		return RobotBase.isSimulation() ? m_state.leftEncoderPosition : m_leftEncoder.getPosition();
	}

	/**
	 * @return The right encoder position (meters)
	 */
	public double getRightEncoderPosition() {
		return RobotBase.isSimulation() ? m_state.rightEncoderPosition : m_rightEncoder.getPosition();
	}

	/**
	 * @return The heading of the gyro (degrees)
	 */
	public double getHeading() {
		return RobotBase.isSimulation() ? m_state.yawInDegrees
				: m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * @return Pose of the robot
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void arcadeDrive(double straight, double left, double right) {
		tankDrive(DriveConstants.kSpeedLimitFactor * (straight - left + right),
				DriveConstants.kSpeedLimitFactor * (straight + left - right));
	}

	public void arcadeDrive(double straight, double turn) {
		tankDrive(DriveConstants.kSpeedLimitFactor * (straight - turn),
				DriveConstants.kSpeedLimitFactor * (straight + turn));
	}

	/**
	 * @param leftSpeed  Left motors percent output
	 * @param rightSpeed Right motors percent output
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		m_frontLeft.set(leftSpeed);
		m_backLeft.set(leftSpeed);
		m_frontRight.set(rightSpeed);
		m_backRight.set(rightSpeed);
		if (RobotBase.isSimulation()) {
			m_state = state(m_state, leftSpeed, rightSpeed);
		}
	}

	private State state(State state, double leftSpeed, double rightSpeed) {
		Function<Double, Double> toDisplacement = (speed) -> 5.0 * speed * TimedRobot.kDefaultPeriod;
		var leftDisplacement = toDisplacement.apply(leftSpeed);
		var rightDisplacement = toDisplacement.apply(rightSpeed);
		var yawDisplacement = Math.toDegrees(
				Math.atan2(rightDisplacement - leftDisplacement, Constants.DriveConstants.kTrackwidthMeters));
		return new State(state.leftEncoderPosition + leftDisplacement, state.rightEncoderPosition + rightDisplacement,
				state.yawInDegrees + yawDisplacement);
	}

	/**
	 * Hacky method that returns if the robot is enabled to bypass a WPILib bug with
	 * loop overruns
	 * 
	 * @author Jonathan Waters
	 * @return Whether or not the robot is enabled
	 */
	public boolean isEnabledFast() {
		DriverStationJNI.getControlWord(m_controlWord);
		return m_controlWord.getEnabled();
	}

	public void setBackBrake() {
		m_backLeft.setIdleMode(IdleMode.kBrake);
		m_backRight.setIdleMode(IdleMode.kBrake);
	}

	public void setBackCoast() {
		m_backLeft.setIdleMode(IdleMode.kCoast);
		m_backRight.setIdleMode(IdleMode.kCoast);
	}

	public void setFrontCoast() {
		m_frontLeft.setIdleMode(IdleMode.kCoast);
		m_frontRight.setIdleMode(IdleMode.kCoast);
	}

	public void setFrontBrake() {
		m_frontLeft.setIdleMode(IdleMode.kBrake);
		m_frontRight.setIdleMode(IdleMode.kBrake);
	}

}
