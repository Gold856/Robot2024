package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleVisionAlignCommand;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.drive.BangBangDriveCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

public class CommandComposer {
	/**
	 * Returns a command to shoot a note and leave the wing (end with LEDS).
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getShootAndLeaveAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				new BangBangDriveDistanceCommand(driveSubsystem, 2, 0.01),
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine at SPEAKER middle.
	 * Shoot a note, drive forward to the next note, and shoot
	 * the note (end with LEDs).
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getTwoScoreMiddleAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new BangBangDriveCommand(driveSubsystem, 1., 90, 0.01));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER right.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute -35 degrees), and shoot the note. End with LEDs.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getTwoScoreRightAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new BangBangDriveCommand(driveSubsystem, 1.2, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, -11, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a two-score autonomous routine on SPEAKER left.
	 * Shoot a note, drive forward to the next note, intake,
	 * turn to align (absolute 35 degrees), and shoot the note. End with LEDs.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * 
	 * @return The command.
	 */
	public static Command getTwoScoreLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new BangBangDriveCommand(driveSubsystem, 1.2, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 45, 2, false),
				new BangBangDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER right.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute 12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getThreeScoreRightAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 12, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 78, 2, false),
				new BangBangDriveCommand(driveSubsystem, .75, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, .5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a three-score autonomous routine on SPEAKER left.
	 * Use two-score routine: Shoot a note, drive forward to the next note,
	 * intake, turn to align, and shoot the note. Play LEDs.
	 * Third note: Turn to middle note, drive forward, intake,
	 * turn to align (absolute -12 degrees), and shoot the note. End with LEDS.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @param visionSubsystem  The vision subsystem with Limelight.
	 *                         If visionSubsystem is used, auto will include command
	 *                         to align with AprilTag.
	 * @return The command.
	 */
	public static Command getThreeScoreLeftAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -12, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAuto(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new BangBangDriveCommand(driveSubsystem, .75, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command to drive back and forth various amounts.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getBlocksAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				DriveDistanceCommand.create(driveSubsystem, 0.75),
				DriveDistanceCommand.create(driveSubsystem, -1.25),
				DriveDistanceCommand.create(driveSubsystem, 1.5),
				DriveDistanceCommand.create(driveSubsystem, -1.825),
				DriveDistanceCommand.create(driveSubsystem, 2.125),
				DriveDistanceCommand.create(driveSubsystem, -2.5),
				new TimedLEDCommand(arduinoSubsystem, 0.5, StatusCode.RAINBOW_PARTY_FUN_TIME));
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

	public static Command getAlignToBlueAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(new Pose(-6.44, 3.5, 90), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(new Pose(-6.44, 3.75, 90), 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getAlignToRedAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(new Pose(6.44, 3.5, 90), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(new Pose(6.44, 3.75, 90), 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getTurnToBlueSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(new Translation2d(-7.87, 1.45), 0.1, 5, driveSubsystem, limeLightSubsystem);
	}

	public static Command getTurnToRedSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(new Translation2d(7.87, 1.45), 0.1, 5, driveSubsystem, limeLightSubsystem);
	}

	public static Command getMoveToBlueSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var p = limeLightSubsystem.estimatedPose();
			var target = p;
			if (p.getY() > 2)
				target = new Pose(-6, 2, 180);
			else if (p.getY() < 0)
				target = new Pose(-6, 0, 180);
			return driveSubsystem.getPose().plus(target.minus(p));
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5)
				.andThen(DriveCommand.turnTo(new Translation2d(-7.87, 1.45), 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}

	public static Command getMoveToRedSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var p = limeLightSubsystem.estimatedPose();
			var target = p;
			if (p.getY() > 2)
				target = new Pose(6, 2, 0);
			else if (p.getY() < 0)
				target = new Pose(6, 0, 0);
			return driveSubsystem.getPose().plus(target.minus(p));
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5)
				.andThen(DriveCommand.turnTo(new Translation2d(7.87, 1.45), 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}

	public static Command getFiveScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(new Pose(-6.5, 0.0, 180), 0.1, 5, driveSubsystem,
				limeLightSubsystem).andThen( // 2nd note
						DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem))
				.andThen( // 3rd note
						DriveCommand.alignTo(
								kBlueNoteTwoPose
										.plus(new Transform2d(new Translation2d(-0.5, 0), Rotation2d.fromDegrees(0))),
								0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem))
				.andThen( // 4th note
						DriveCommand.alignTo(
								kBlueNoteThreePose
										.plus(new Transform2d(new Translation2d(-0.5, 0), Rotation2d.fromDegrees(0))),
								0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem))
				.andThen( // 5th note
						DriveCommand.alignTo(
								kCenterNoteOnePose
										.plus(new Transform2d(new Translation2d(-0.5, 0), Rotation2d.fromDegrees(0))),
								0.3, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(new Pose(-6, 2.5, -155), 0.2, 10, driveSubsystem,
								limeLightSubsystem))
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}
}
