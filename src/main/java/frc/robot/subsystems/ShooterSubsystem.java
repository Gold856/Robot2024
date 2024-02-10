package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import java.time.Duration;
// import java.time.Instant;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoShooterMaster = new CANSparkMax(ShooterConstants.kShooterMasterPort,
			MotorType.kBrushless);
	private final CANSparkMax m_neoShooterFollower = new CANSparkMax(ShooterConstants.kShooterFollowerPort,
			MotorType.kBrushless);
	private final SparkPIDController m_neoController = m_neoShooterMaster.getPIDController();
	private final RelativeEncoder m_neoEncoderMaster = m_neoShooterMaster.getEncoder();
	private double m_setVelocity;
	private double m_speakerHeight = Constants.ShooterConstants.speakerHeight;
	private double m_kShooterTolerance = Constants.ShooterConstants.kShooterTolerance;
	private double m_actuatorHeightSetpoint = 0;

	// private Instant m_startTime;
	/**
	 * Initializes a new instance of the {@link FlywheelSubsystem} class.
	 */
	public ShooterSubsystem() {
		// Initialize Motors
		m_neoShooterMaster.restoreFactoryDefaults();
		m_neoShooterMaster.setInverted(ShooterConstants.kMasterInvert);
		m_neoShooterMaster.setIdleMode(IdleMode.kCoast);
		m_neoShooterMaster.enableVoltageCompensation(12);
		m_neoShooterMaster.setSmartCurrentLimit(ShooterConstants.kSmartCurrentLimit);
		m_neoShooterMaster.setSecondaryCurrentLimit(ShooterConstants.kPeakCurrentLimit,
				ShooterConstants.kPeakCurrentDurationMillis);

		m_neoShooterFollower.restoreFactoryDefaults();
		m_neoShooterFollower.setIdleMode(IdleMode.kCoast);
		m_neoShooterFollower.enableVoltageCompensation(12);
		m_neoShooterFollower.setSmartCurrentLimit(ShooterConstants.kSmartCurrentLimit);
		m_neoShooterFollower.setSecondaryCurrentLimit(ShooterConstants.kPeakCurrentLimit,
				ShooterConstants.kPeakCurrentDurationMillis);
		m_neoShooterFollower.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.kLeadScrewLimit);
		m_neoShooterFollower.follow(m_neoShooterMaster, ShooterConstants.kFollowerOppose);

		m_neoEncoderMaster.setPositionConversionFactor(ShooterConstants.kGearRatio);
		m_neoEncoderMaster.setVelocityConversionFactor(ShooterConstants.kGearRatio);
		float leadScrewSoftLimit = (float) (ShooterConstants.kLeadScrewLimit
				* m_neoEncoderMaster.getPositionConversionFactor());
		m_neoShooterMaster.setSoftLimit(SoftLimitDirection.kForward, leadScrewSoftLimit);

		m_neoController.setP(ShooterConstants.kP);
		m_neoController.setI(ShooterConstants.kI);
		m_neoController.setD(ShooterConstants.kD);
		m_neoController.setIZone(ShooterConstants.kIz);
		m_neoController.setFF(ShooterConstants.kFF);
		m_neoController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
	}

	public void periodic() {
	}

	/**
	 * @return Measured velocity.
	 */
	public double getVelocity() {
		return m_neoEncoderMaster.getVelocity();
	}

	/**
	 * Sets target speed for shooter.
	 * 
	 * @param velocity Target velocity (rpm).
	 */
	public void setVelocity(double velocity) {
		m_setVelocity = velocity;
		m_neoController.setReference(m_setVelocity, ControlType.kVelocity, 0);
	}

	public void stopMotors() {
		setVelocity(0);
	}

	public double calculateVelocity(double distanceToSpeaker) {
		double vI = (distanceToSpeaker * 9.8)
				/ (Math.cos(calculateAngle(distanceToSpeaker)) * (Math.sin(calculateAngle(distanceToSpeaker))));
		return vI;
	}

	public void setVelocityForNegatives() {
		m_setVelocity = -5;
	}

	public double calculateAngle(double distanceToSpeaker) {
		double angle = Math.atan2(m_speakerHeight, distanceToSpeaker);
		return angle;
	}

	public double getActuatorHeight() {
		double actuatorHeight = 0; // m_encoder.getPosition();
		return actuatorHeight; // TODO fix this
	}

	public void setActuatorHeight(double actuatorHeightSetpoint) {
		setVelocity(1500);
		// m_startTime = Instant.now();
		m_setPosition = position;
		m_pidController.setReference(position, CANSparkMax.ControlType.kPosition, HoodConstants.kSlotID);
		m_actuatorHeightSetpoint = actuatorHeightSetpoint;
	}

	public boolean atActuatorSetpoint() {
		return (Math.abs(m_actuatorHeightSetpoint - getActuatorHeight()) <= m_kShooterTolerance);
	}

	public void configureShuffleboard(boolean inCompetitionMode) {
		if (!inCompetitionMode) {
			ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Flywheel");
			shuffleboardTab.addNumber("Flywheel Velocity", () -> getVelocity()).withSize(4, 2).withPosition(0, 0)
					.withWidget(BuiltInWidgets.kGraph);
			shuffleboardTab.addBoolean("At setpoint", () -> atActuatorSetpoint()).withSize(1, 1).withPosition(0, 2)
					.withWidget(BuiltInWidgets.kBooleanBox);
		}
	}
}