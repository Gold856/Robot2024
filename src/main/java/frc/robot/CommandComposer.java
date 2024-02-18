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
		return sequence(
				// new SetSteeringCommand(driveSubsystem, 0),
				new BangBangDriveCommand(driveSubsystem, 1.2, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, -11, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
	}

	/**
	 * Returns a command to shoot a note, drive forward to the next note, and shoot
	 * the note.
	 * 
	 * @param driveSubsystem The drive subsystem.
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAuto(DriveSubsystem driveSubsystem, SimpleVisionSubsystem visionSubsystem) {
		return sequence(
				new BangBangDriveCommand(driveSubsystem, 1.2, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 11, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
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
