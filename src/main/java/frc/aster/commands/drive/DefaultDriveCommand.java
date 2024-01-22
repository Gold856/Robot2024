package frc.aster.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.aster.Constants.ControllerConstants;
import frc.aster.Constants.DriveConstants;
import frc.aster.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends Command {
	private final Supplier<Double> m_speedStraight, m_speedLeft, m_speedRight;

	/**
	 * Drive using speed inputs as a percentage output of the motor
	 * 
	 * @param driveSubsystem The subsystem to be used
	 * @param speedStraight  Joystick input
	 * @param speedLeft      Left Trigger input
	 * @param speedRight     Right Trigger input
	 */
	public DefaultDriveCommand(Supplier<Double> speedStraight, Supplier<Double> speedLeft,
			Supplier<Double> speedRight) {
		m_speedStraight = speedStraight;
		m_speedLeft = speedLeft;
		m_speedRight = speedRight;
		addRequirements(DriveSubsystem.get());
	}

	/** Update the motor outputs */
	public void execute() {
		// Apply deadbands to controller input so it doesn't move while the controller
		// isn't touched
		double speedStraight = MathUtil.applyDeadband(m_speedStraight.get(), ControllerConstants.kDeadzone);
		double speedLeft = MathUtil.applyDeadband(m_speedLeft.get(), ControllerConstants.kTriggerDeadzone);
		double speedRight = MathUtil.applyDeadband(m_speedRight.get(), ControllerConstants.kTriggerDeadzone);
		// Full turn speed is difficult to control, so we slow it down
		speedLeft *= DriveConstants.kTurningMultiplier;
		speedRight *= DriveConstants.kTurningMultiplier;
		DriveSubsystem.get().arcadeDrive(speedStraight, speedLeft, speedRight);
	}
}
