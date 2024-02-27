package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kIntakePort, MotorType.kBrushless);
	private final RelativeEncoder m_encoder = m_motor.getEncoder();
	private final SparkPIDController m_controller = m_motor.getPIDController();

	/**
	 * Initializes a new instance of the {@link IntakeSubsystem} class.
	 */
	public IntakeSubsystem() {
		m_motor.restoreFactoryDefaults();
		m_motor.setIdleMode(IdleMode.kCoast);
		m_motor.enableVoltageCompensation(12);
		m_motor.setSmartCurrentLimit(kSmartCurrentLimit);
		m_motor.setSecondaryCurrentLimit(kPeakCurrentLimit);
		m_encoder.setVelocityConversionFactor(kIntakeGearRatio);
		m_controller.setP(IntakeConstants.kP);
		m_controller.setI(IntakeConstants.kI);
		m_controller.setD(IntakeConstants.kD);
		m_controller.setIZone(IntakeConstants.kIz);
		m_controller.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
	}

	/**
	 * Sets the speed of the intake motor.
	 * 
	 * @param speed The speed.
	 */
	public void setSpeed(double speed) {
		m_motor.set(speed);
	}

	/**
	 * Sets the speed of the intake motor.
	 * 
	 * @param velocity The velocity (rpm).
	 */
	public void setVelocity(double velocity) {
		m_controller.setReference(velocity, ControlType.kVelocity);
	}

	/**
	 * Creates a command to reverse the intake.
	 * 
	 * @return The command.
	 */
	public Command reverseIntakeCommand() {
		return runOnce(() -> setSpeed(.1));
	}

	/**
	 * Creates a command to spin the intake forward.
	 * 
	 * @return The command.
	 */
	public Command forwardIntakeCommand() {
		return runOnce(() -> setSpeed(-0.75));
	}

	/**
	 * Creates a command to stop the intake.
	 * 
	 * @return The command.
	 */
	public Command stopIntakeCommand() {
		return runOnce(() -> setSpeed(0));
	}

	public void periodic() {
		SmartDashboard.putNumber("Intake Current", m_motor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Velocity", m_encoder.getVelocity());
	}
}
