package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter.RegressionTargeter;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.AimerSubsystem.AimHeightOperation;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

// TODO UPDATE DOCUMENTATION
// TODO All the autos (update so it is not using the AimAndShoot teleop one)

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static ArduinoSubsystem m_arduinoSubsystem;
	private static PneumaticsSubsystem m_pneumaticsSubsystem;
	private static AimerSubsystem m_aimerSubsystem;
	private static RegressionTargeter m_targeter;
	private static IndexerSubsystem m_indexerSubsystem;
	private static SimpleVisionSubsystem m_simpleVisionSubsystem;
	private static FlywheelSubsystem m_flywheelSubsystem;
	private static IntakeSubsystem m_intakeSubsystem;
	private static LimeLightSubsystem m_limeLightSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, AimerSubsystem aimerSubsystem, RegressionTargeter targeter,
			IndexerSubsystem indexerSubsystem, SimpleVisionSubsystem simpleVisionSubsystem,
			FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_arduinoSubsystem = arduinoSubsystem;
		m_pneumaticsSubsystem = pneumaticsSubsystem;
		m_aimerSubsystem = aimerSubsystem;
		m_targeter = targeter;
		m_indexerSubsystem = indexerSubsystem;
		m_simpleVisionSubsystem = simpleVisionSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;
		m_intakeSubsystem = intakeSubsystem;
		m_limeLightSubsystem = limeLightSubsystem;
	}

	public static Command getThreeScoreAuto() {
		final var factory = m_driveSubsystem.autoFactory();

		final var speakerToLeftNote = factory.trajectory("Better 3 score auto", 0, factory.voidLoop());
		final var leftNoteToLaunch = factory.trajectoryCommand("Better 3 score auto", 1);
		final var launchToRightNote = factory.trajectoryCommand("Better 3 score auto", 2);
		final var rightNoteToLaunch = factory.trajectoryCommand("Better 3 score auto", 3);
		return sequence(
				parallel(
						sequence(
								m_driveSubsystem.resetOdometryCommand(
										speakerToLeftNote.getInitialPose().orElse(new Pose2d())),
								speakerToLeftNote.cmd()),
						// m_pneumaticsSubsystem.downIntakeCommand(),
						m_intakeSubsystem.forwardIntakeCommand()),
				m_intakeSubsystem.stopIntakeCommand(),
				parallel(
						leftNoteToLaunch,
						// TODO: Figure out good height
						m_aimerSubsystem.aimHeightCommand(AimHeightOperation.PRESET_PODIUM,
								m_targeter),
						// TODO: Figure out good speed
						m_flywheelSubsystem.spinCommand(8000, 8000)),
				sequence(
						new IndexWithSensorCommand(m_indexerSubsystem, 0.5),
						launchToRightNote),
				m_intakeSubsystem.forwardIntakeCommand(),
				rightNoteToLaunch,
				m_intakeSubsystem.stopIntakeCommand(),
				new IndexWithSensorCommand(m_indexerSubsystem, 0.5)); // Already set up aimer and flywheel
		// return sequence(
		// m_driveSubsystem.resetOdometryCommand(speakerToLeftNote.getInitialPose().orElse(new
		// Pose2d())),
		// speakerToLeftNote.cmd(),
		// leftNoteToLaunch,
		// launchToRightNote,
		// rightNoteToLaunch);
	}

	public static Command getIntakeWithSensorCommand() {
		return sequence(
				parallel(
						new IndexWithSensorCommand(m_indexerSubsystem, 0.5),
						m_intakeSubsystem.forwardIntakeCommand(),
						m_arduinoSubsystem.writeStatus(StatusCode.SOLID_ORANGE)),
				m_intakeSubsystem.stopIntakeCommand(),
				m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
	}

	public static Command getTeleopIntakeCommand() {
		return sequence(
				m_pneumaticsSubsystem.downIntakeCommand(),
				getIntakeWithSensorCommand(),
				m_pneumaticsSubsystem.upIntakeCommand());
	}
}
