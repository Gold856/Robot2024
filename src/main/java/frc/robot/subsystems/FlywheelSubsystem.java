package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoFlywheelMaster = new CANSparkMax(kMasterPort, MotorType.kBrushless);
	private final CANSparkMax m_neoFlywheelFollower = new CANSparkMax(kFollowerPort, MotorType.kBrushless);
	private final SparkPIDController m_neoController = m_neoFlywheelMaster.getPIDController();
	private final RelativeEncoder m_neoEncoderMaster = m_neoFlywheelMaster.getEncoder();

	private double m_setVelocity;

	/**
	 * Initializes a new instance of the {@link FlywheelSubsystem} class.
	 */
	public FlywheelSubsystem() {
		// Initialize Motors
		m_neoFlywheelMaster.restoreFactoryDefaults();
		m_neoFlywheelMaster.setInverted(kMasterInvert);
		m_neoFlywheelMaster.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelMaster.enableVoltageCompensation(12);
		m_neoFlywheelMaster.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoFlywheelMaster.setSecondaryCurrentLimit(kPeakCurrentLimit,
				kPeakCurrentDurationMillis);

		m_neoFlywheelFollower.restoreFactoryDefaults();
		m_neoFlywheelFollower.setIdleMode(IdleMode.kCoast);
		m_neoFlywheelFollower.enableVoltageCompensation(12);
		m_neoFlywheelFollower.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoFlywheelFollower.setSecondaryCurrentLimit(kPeakCurrentLimit,
				kPeakCurrentDurationMillis);
		m_neoFlywheelFollower.follow(m_neoFlywheelMaster, kFollowerOppose);

		m_neoEncoderMaster.setVelocityConversionFactor(1 / kGearRatio);

		m_neoController.setP(kP);
		m_neoController.setI(kI);
		m_neoController.setD(kD);
		m_neoController.setIZone(kIz);
		m_neoController.setFF(kFF);
		m_neoController.setOutputRange(kMinOutput, kMaxOutput);
	}

	public void periodic() {
		SmartDashboard.putBoolean("Flywheel at Setpoint", atSetpoint());
		// SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
		// SmartDashboard.putNumber("Flywheel Setpoint", m_setVelocity);
		// essentially the end method of the flywheel velocity setpoint mode
		if (m_setVelocity == 0 && Math.abs(m_neoEncoderMaster.getVelocity()) > 0.05) {
			m_neoFlywheelMaster.stopMotor();
		}
	}

	public void setSpeed(double reverse) {
		m_neoFlywheelMaster.set(reverse);
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
		m_setVelocity = velocity;
		m_neoController.setReference(m_setVelocity, ControlType.kVelocity);
	}

	/**
	 * @return Whether the flywheel is at its setpoint ABOVE 0
	 */
	public boolean atSetpoint() {
		return Math.abs(getVelocity() - getSetpoint()) < kAllowedError;
	}
}