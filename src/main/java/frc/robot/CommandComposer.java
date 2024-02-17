package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.BangBangDriveCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.SetSteeringCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem;

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
		return sequence(driveSubsystem.offsetGyroCommand(-60),
				new SetSteeringCommand(driveSubsystem, 0),
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
}
