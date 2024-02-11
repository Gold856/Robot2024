package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private final CANSparkMax m_neoIntakeMaster = new CANSparkMax(kMasterPort, MotorType.kBrushless);
	private final CANSparkMax m_neoIntakeFollower = new CANSparkMax(kFollowerPort, MotorType.kBrushless);
	private final RelativeEncoder m_neoEncoderMaster = m_neoIntakeMaster.getEncoder();

	/**
	 * Initializes a new instance of the {@link IntakeSubsystem} class.
	 */
	public IntakeSubsystem() {
		// Initialize Motors
		m_neoIntakeMaster.restoreFactoryDefaults();
		m_neoIntakeMaster.setIdleMode(IdleMode.kCoast);
		m_neoIntakeMaster.enableVoltageCompensation(12);
		m_neoIntakeMaster.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoIntakeMaster.setSecondaryCurrentLimit(kPeakCurrentLimit);

		m_neoIntakeFollower.restoreFactoryDefaults();
		m_neoIntakeFollower.setIdleMode(IdleMode.kCoast);
		m_neoIntakeFollower.enableVoltageCompensation(12);
		m_neoIntakeFollower.setSmartCurrentLimit(kSmartCurrentLimit);
		m_neoIntakeFollower.setSecondaryCurrentLimit(kPeakCurrentLimit);
		m_neoIntakeFollower.follow(m_neoIntakeMaster, kFollowerOppose);

		m_neoEncoderMaster.setVelocityConversionFactor(1 / kGearRatio);
	}

	/**
	 * Sets the speed of the intake motor.
	 * 
	 * @param speed The speed.
	 */
	public void setSpeed(double speed) {
		m_neoIntakeMaster.set(speed);
	}

	/**
	 * Creates a command to reverse the intake.
	 * 
	 * @return The command.
	 */
	public Command reverseIntakeCommand() {
		return runOnce(() -> setSpeed(-.1));
	}

	/**
	 * Creates a command to spin the intake forward.
	 * 
	 * @return The command.
	 */
	public Command forwardIntakeCommand() {
		return run(() -> setSpeed(.1));
	}

	/**
	 * Creates a command to stop the intake.
	 * 
	 * @return The command.
	 */
	public Command stopIntakeCommand() {
		return run(() -> setSpeed(0));
	}
}