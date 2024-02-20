package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClampedController;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoShooter = new CANSparkMax(ShooterConstants.kShooterLeadScrewPort,
			MotorType.kBrushless);
	private final ClampedController m_controller = new ClampedController(.75, 0.05, kMaxShooterPower);
	private final CANcoder m_aimCancoder = new CANcoder(ShooterConstants.kShooterEncoderPort);
	private double m_actuatorHeightSetpoint;
	private boolean m_isManual;

	/**
	 * Initializes a new instance of the {@link ShooterSubsystem} class.
	 */
	public ShooterSubsystem() {
		// Initialize Motors
		m_neoShooter.restoreFactoryDefaults();
		m_neoShooter.setInverted(ShooterConstants.kMasterInvert);
		m_neoShooter.setIdleMode(IdleMode.kBrake);
		m_neoShooter.enableVoltageCompensation(12);
		m_neoShooter.setSmartCurrentLimit(ShooterConstants.kSmartCurrentLimit);
		m_neoShooter.setSecondaryCurrentLimit(ShooterConstants.kPeakCurrentLimit,
				ShooterConstants.kPeakCurrentDurationMillis);
	}

	public void periodic() {
		if (!m_isManual) {
			if (!m_controller.atSetpoint()) {
				m_neoShooter.set(m_controller.calculate(getActuatorHeight()));
			} else {
				m_neoShooter.set(0);
			}
		}
	}

	public void stopMotor() {
		setSpeed(0);
	}

	public double calcActuatorHeightFromDistance(double distanceMeters) {
		return (ShooterConstants.shooterLength * ShooterConstants.speakerHeight) / distanceMeters;
	}

	// TODO: Returns 0 - 1.0
	public double getActuatorHeight() {
		return m_aimCancoder.getAbsolutePosition().getValueAsDouble() / ShooterConstants.kShooterMaxEncoderValue;
	}

	// TODO: document 0-1.0
	public void setActuatorHeight(double actuatorHeightSetpoint) {
		m_controller.setSetpoint(actuatorHeightSetpoint);

	}

	public void adjustActuatorSetpoint(double adjustAmount) {
		m_actuatorHeightSetpoint += adjustAmount;
		setActuatorHeight(m_actuatorHeightSetpoint);
	}

	public void setSpeed(double speed) {
		speed = Math.signum(speed) * MathUtil.clamp(Math.abs(speed), 0, ShooterConstants.kMaxShooterPower);
		m_neoShooter.set(speed);
	}

	public boolean atActuatorSetpoint() {
		return m_controller.atSetpoint();
	}

	public void setManual(boolean isManual) {
		m_isManual = isManual;
	}
}