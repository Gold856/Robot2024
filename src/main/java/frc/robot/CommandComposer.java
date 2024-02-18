package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SimpleVisionAlignCommand;
import frc.robot.commands.drive.BangBangDriveCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

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
	public static Command getTwoScoreRightAuto(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		Command alignCommand;
		if (visionSubsystem == null) {
			alignCommand = new TurnToAngleCommand(driveSubsystem, -35, 2, false);
		} else {
			alignCommand = new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem);
		}
		return sequence(
				// new SetSteeringCommand(driveSubsystem, 0),
				new BangBangDriveCommand(driveSubsystem, 1.2, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, -11, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand);
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAuto(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		Command alignCommand;
		if (visionSubsystem == null) {
			alignCommand = new TurnToAngleCommand(driveSubsystem, 35, 2, false);
		} else {
			alignCommand = new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem);
		}
		return sequence(
				new BangBangDriveCommand(driveSubsystem, 1.2, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 45, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand);
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getThreeScoreRightAuto(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		Command alignCommand;
		if (visionSubsystem == null) {
			alignCommand = new TurnToAngleCommand(driveSubsystem, 12, 2, false);
		} else {
			alignCommand = new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem);
		}
		return sequence(
				// right note
				getTwoScoreRightAuto(driveSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 78, 2, false),
				new BangBangDriveCommand(driveSubsystem, .75, 180, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				alignCommand);
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getThreeScoreLeftAuto(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		Command alignCommand;
		if (visionSubsystem == null) {
			alignCommand = new TurnToAngleCommand(driveSubsystem, -12, 2, false);
		} else {
			alignCommand = new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem);
		}
		return sequence(
				// right note
				getTwoScoreLeftAuto(driveSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new BangBangDriveCommand(driveSubsystem, .75, -180, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				alignCommand);
	}

	public static Command getBlocksAuto(DriveSubsystem driveSubsystem) {
		return sequence(
				DriveDistanceCommand.create(driveSubsystem, 0.75),
				DriveDistanceCommand.create(driveSubsystem, -1.25),
				DriveDistanceCommand.create(driveSubsystem, 1.5),
				DriveDistanceCommand.create(driveSubsystem, -1.825),
				DriveDistanceCommand.create(driveSubsystem, 2.125),
				DriveDistanceCommand.create(driveSubsystem, -2.5));
	}
}
