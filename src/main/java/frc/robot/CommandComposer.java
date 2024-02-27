package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleVisionAlignCommand;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DrivePathCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
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
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
				new PolarDriveCommand(driveSubsystem, 1., 90, 0.01));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getTwoScoreRightAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 0.75, 180, 0.01),
				new PolarDriveCommand(driveSubsystem, 1.2, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getTwoScoreRightAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 1.4, 240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getTwoScoreLeftAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 25, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 1.2, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getTwoScoreLeftAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 35, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new PolarDriveCommand(driveSubsystem, 0.75, -180, 0.01),
				new PolarDriveCommand(driveSubsystem, 1, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.2, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getThreeScoreRightAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, .25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getThreeScoreRightAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAutoRed(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, .25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getThreeScoreLeftAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getThreeScoreLeftAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getFourScoreRightAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 30, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 2, 310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .5, 180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
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
	public static Command getFourScoreRightAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 40, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoRed(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 1.5, 290, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, 180, 0.01),
				new PolarDriveCommand(driveSubsystem, -0.2, -180),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a four-score autonomous routine on SPEAKER left.
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
	public static Command getFourScoreLeftAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {

			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 2, -310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, -180, 0.01),
				new PolarDriveCommand(driveSubsystem, -0.2, -180),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for a four-score autonomous routine on SPEAKER left.
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
	public static Command getFourScoreLeftAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -20, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, visionSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 1.8, -280, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .35, -180, 0.01),
				alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * 
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getAmpTwoAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 0.22, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.4, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * 
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getAmpTwoAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 0.22, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				new TimedLEDCommand(arduinoSubsystem, 0.4, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * Returns a command for
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getShootAndAmp(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 1.4, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				// strafe away, intake
				new TurnToAngleCommand(driveSubsystem, -90, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.70, 90),
				new PolarDriveCommand(driveSubsystem, -1.05, 180));
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
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getAlignToBlueAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueAmpPose.add(new Pose(0, -0.3, 0)), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(kBlueAmpPose, 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getAlignToRedAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kRedAmpPose.add(new Pose(0, -0.3, 0)), 0.2, 10, driveSubsystem, limeLightSubsystem)
				.andThen(DriveCommand.alignTo(kRedAmpPose, 0.1, 5, driveSubsystem, limeLightSubsystem));
	}

	public static Command getTurnToBlueSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem, limeLightSubsystem);
	}

	public static Command getTurnToRedSpeaker(DriveSubsystem driveSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.turnTo(kRedSpeakerPosition, 0.1, 5, driveSubsystem, limeLightSubsystem);
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
				.andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
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
				.andThen(DriveCommand.turnTo(kRedSpeakerPosition, 0.1, 5, driveSubsystem,
						limeLightSubsystem));
	}

	public static Command getFiveScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return
		// DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem)
		// 2nd note
		DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				.andThen( // 3rd note
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen( // 4th note
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(
								kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), 0.2, 10, driveSubsystem,
								limeLightSubsystem))
				.andThen(// 5th note
						DriveCommand.alignTo(kBlueCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFiveScoreBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return
		// DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem)
		// 2nd note
		DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				.andThen( // 3rd note
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen( // 4th note
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen( // 5th note
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.2,
								10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteTwoPose.add(new Pose(-0.5, 0.5, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.1,
								5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteTwoPose.add(new Pose(-0.5, 0.5, 0)), kBlueCenterNoteTwoPose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleCenterBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteOnePose.add(new Pose(-2, 0, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose.add(new Pose(-0.5, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteThreePose.add(new Pose(-2.75, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleCenterBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteTwoPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteOnePose), 0.1,
								5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteOnePose.add(new Pose(-2, 0, 0)), kBlueNoteTwoPose), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteThreePose.add(new Pose(-0.5, 0, 0)), kBlueCenterNoteThreePose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteThreePose.add(new Pose(-2.75, 0, 0)), kBlueNoteTwoPose), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBottomBlueAutoCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(0, -1, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFourPose.add(new Pose(-.3, -.3, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFourPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFivePose.add(new Pose(-.5, 0, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueCenterNoteFivePose, 0.1, 5, driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)), 0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DriveCommand.alignTo(kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0)), 0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
	}

	public static Command getFourScoreMiddleBottomBlueAutoCommandOptimized(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		return DriveCommand.alignTo(kBlueNoteThreePose, 0.1, 5, driveSubsystem,
				limeLightSubsystem)
				// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
				// limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueNoteThreePose.add(new Pose(0, -1, 0)),
										kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
										kBlueCenterNoteFourPose.add(new Pose(-.3, -.3, 0)), kBlueCenterNoteFourPose),
								0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
										kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0))),
								0.1, 5,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(
								List.of(kBlueCenterNoteFivePose.add(new Pose(-.5, 0, 0)), kBlueCenterNoteFivePose),
								0.2, 10,
								driveSubsystem,
								limeLightSubsystem))
				.andThen(
						DrivePathCommand.passThrough(List.of(kBlueNoteThreePose.add(new Pose(2.3, -2.5, 0)),
								kBlueNoteThreePose.add(new Pose(1.5, -1.5, 0))), 0.2, 10, driveSubsystem,
								limeLightSubsystem))
		// .andThen(DriveCommand.turnTo(kBlueSpeakerPosition, 0.1, 5, driveSubsystem,
		// limeLightSubsystem))
		;
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

	public static Command getBallPathTest(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
			FlywheelSubsystem flywheelSubsystem) {
		return parallel(
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 3000),
				intakeSubsystem.forwardIntakeCommand(),
				IndexerCommand.getFowardCommand(indexerSubsystem));
	}
}
