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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoFlywheelMaster = new CANSparkMax(FlywheelConstants.kMasterPort,
			MotorType.kBrushless);
	private final CANSparkMax m_neoFlywheelFollower = new CANSparkMax(FlywheelConstants.kFollowerPort,
			MotorType.kBrushless);
	private final SparkPIDController m_neoController = m_neoFlywheelMaster.getPIDController();
	private final RelativeEncoder m_neoEncoderMaster = m_neoFlywheelMaster.getEncoder();
	private double m_setVelocity;
	private double m_speakerHeight = Constants.ShooterConstants.speakerHeight;
	private double m_shooterLength = Constants.ShooterConstants.shooterLength;

	// private Instant m_startTime;
	/**
	 * Initializes a new instance of the {@link FlywheelSubsystem} class.
	 */
	public ShooterSubsystem() {
		// Initialize Motors
		m_neoFlywheelMaster.restoreFactoryDefaults();
		m_neoFlywheelMaster.setInverted(FlywheelConstants.kMasterInvert);
		m_neoFlywheelMaster.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelMaster.enableVoltageCompensation(12);
		m_neoFlywheelMaster.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
		m_neoFlywheelMaster.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
				FlywheelConstants.kPeakCurrentDurationMillis);
		m_neoFlywheelMaster.setSoftLimit(SoftLimitDirection.kForward, 0.0f);

		m_neoFlywheelFollower.restoreFactoryDefaults();
		m_neoFlywheelFollower.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelFollower.enableVoltageCompensation(12);
		m_neoFlywheelFollower.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
		m_neoFlywheelFollower.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
				FlywheelConstants.kPeakCurrentDurationMillis);
		m_neoFlywheelFollower.follow(m_neoFlywheelMaster, FlywheelConstants.kFollowerOppose);

		m_neoEncoderMaster.setPositionConversionFactor(1 / FlywheelConstants.kGearRatio);
		m_neoEncoderMaster.setVelocityConversionFactor(1 / FlywheelConstants.kGearRatio);

		m_neoController.setP(FlywheelConstants.kP);
		m_neoController.setI(FlywheelConstants.kI);
		m_neoController.setD(FlywheelConstants.kD);
		m_neoController.setIZone(FlywheelConstants.kIz);
		m_neoController.setFF(FlywheelConstants.kFF);
		m_neoController.setOutputRange(FlywheelConstants.kMinOutput, FlywheelConstants.kMaxOutput);
	}

	public void periodic() {
		SmartDashboard.putBoolean("Flywheel at Setpoint", atSetpoint());
		// SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
		// SmartDashboard.putNumber("Flywheel Setpoint", m_setVelocity);
		// essentially the end method of the flywheel velocity setpoint mode
		if (m_setVelocity == 0 && Math.abs(m_neoEncoderMaster.getVelocity()) > 0.05) {
			m_neoFlywheelMaster.stopMotor();
		} else {
			// m_neoController.setReference(m_setVelocity, ControlType.kVelocity, 0);
			// m_neoFlywheelMaster.set(neoBangBangController.calculate(m_neoEncoderMaster.getVelocity(),
			// m_setVelocity));
		}

	}

	public void setSpeed(double speed) {
		m_neoFlywheelMaster.set(speed);
	}

	public void incrementSpeed() {
		setVelocity(m_setVelocity + 50);
	}

	public void decrementSpeed() {
		setVelocity(m_setVelocity - 50);
	}

	/**
	 * @return Current setpoint.
	 */
	public double getSetpoint() {
		return m_setVelocity;
	}

	/**
	 * @return Measured velocity.
	 */
	public double getVelocity() {
		return m_neoEncoderMaster.getVelocity();
	}

	/**
	 * Sets target speed for flywheel.
	 * 
	 * @param velocity Target velocity (rpm).
	 */
	public void setVelocity(double velocity) {
		// System.out.println("VELOCITY:" + velocity);
		// m_startTime = Instant.now();
		m_setVelocity = velocity;
		m_neoController.setReference(m_setVelocity, ControlType.kVelocity, 0);
	}

	// public double calcActuatorHeightFromDistance(double distanceToSpeaker) { //
	// actuatorHeight is height of linear
	// // actuator
	// double actuatorHeight = (m_shooterLength * m_speakerHeight) /
	// distanceToSpeaker;
	// return actuatorHeight;
	// }

	// public double calcActuatorHeightFromAngle(double angle) {
	// double actuatorHeight = m_shooterLength / (Math.tan(angle));
	// return actuatorHeight;
	// }

	public void setActuatorHeight(double actuatorHeightSetpoint) {
		// double height = calculateActuatorHeight(distance);
		// TODO: set small delta y
	}

	public double calculateAngle(double distanceToSpeaker) {
		double angle = Math.atan2(m_speakerHeight, distanceToSpeaker);
		return angle;
	}

	public double calculateVelocity(double distanceToSpeaker) {
		double vI = (distanceToSpeaker * 9.8)
				/ (Math.cos(calculateAngle(distanceToSpeaker)) * (Math.sin(calculateAngle(distanceToSpeaker))));
		return vI;

	}

	public void setVelocityForNegatives() {
		m_setVelocity = -5;
	}

	/**
	
	 */
	public boolean atSetpoint() {
		return Math.abs(getVelocity() - getSetpoint()) < 50;
	}

	public void configureShuffleboard(boolean inCompetitionMode) {
		if (!inCompetitionMode) {
			ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Flywheel");
			shuffleboardTab.addNumber("Flywheel Velocity", () -> getVelocity()).withSize(4, 2).withPosition(0, 0)
					.withWidget(BuiltInWidgets.kGraph);
			shuffleboardTab.addBoolean("At setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(0, 2)
					.withWidget(BuiltInWidgets.kBooleanBox);
			// shuffleboardTab.addNumber("Current draw", () ->
			// m_neoFlywheelMaster.getOutputCurrent() +
			// m_neoFlywheelFollower.getOutputCurrent());
			// shuffleboardTab.addNumber("Setpoint", () ->
			// getSetpoint()).withWidget(BuiltInWidgets.kTextView).withSize(1,
			// 1).withPosition(5, 1);
		}
	}
}