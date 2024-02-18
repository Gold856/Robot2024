package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.BangBangDriveCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class CommandComposer {
	/**
	 * Returns a command to shoot a note and leave the wing.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getShootAndLeaveAuto(DriveSubsystem driveSubsystem) {
		return sequence(new BangBangDriveDistanceCommand(driveSubsystem, 2, 0.01));
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getTwoScoreMiddleAuto(DriveSubsystem driveSubsystem) {
		return new BangBangDriveCommand(driveSubsystem, 1., 90, 0.01);
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getTwoScoreRightAuto(DriveSubsystem driveSubsystem) {
		return sequence(
				// new SetSteeringCommand(driveSubsystem, 0),
				new BangBangDriveCommand(driveSubsystem, 1.4, 60, 0.01),
				new TurnToAngleCommand(driveSubsystem, -27, 2, false));
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAuto(DriveSubsystem driveSubsystem) {
		return new BangBangDriveCommand(driveSubsystem, 1., 90, 0.01);
	}

	/**
	 * Returns a command to drive back and forth as described.
	 * 
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getBlocksAuto(DriveSubsystem driveSubsystem) {
		return sequence(
				DriveDistanceCommand.create(driveSubsystem, 0.75),
				DriveDistanceCommand.create(driveSubsystem, -1.25),
				DriveDistanceCommand.create(driveSubsystem, 1.5),
				DriveDistanceCommand.create(driveSubsystem, -1.825),
				DriveDistanceCommand.create(driveSubsystem, 2.125),
				DriveDistanceCommand.create(driveSubsystem, -2.5));
	}

	public static Command getIntakeWithSensorCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				parallel(
						new IndexWithSensorCommand(indexerSubsystem, 0.5),
						intakeSubsystem.forwardIntakeCommand(),
						arduinoSubsystem.writeStatus(StatusCode.SOLID_ORANGE)),
				intakeSubsystem.stopIntakeCommand(),
				arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
	}

	public static Command getTeleopIntakeCommand(IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, IndexerSubsystem indexerSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				pneumaticsSubsystem.upIntakeCommand());
	}
}
