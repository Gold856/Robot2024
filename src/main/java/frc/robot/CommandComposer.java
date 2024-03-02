package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleVisionAlignCommand;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.aimshooter.AimHeightCommand;
import frc.robot.commands.aimshooter.AimHeightCommand.AimHeightOperation;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.subsystems.AimerSubsystem;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
	public static Command getShootAndLeaveAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			AimerSubsystem aimerSubsystem, Targeter targeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		return sequence(
				new AimHeightCommand(aimerSubsystem, targeter, AimHeightOperation.PRESET_SUBWOOFER),
				new IndexerShootCommand(indexerSubsystem),
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
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
			AimerSubsystem aimerSubsystem, Targeter targeter, PneumaticsSubsystem pneumaticsSubsystem,
			SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
			FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new PolarDriveCommand(driveSubsystem, 1., 90, 0.01));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				new AimHeightCommand(aimerSubsystem, targeter, AimHeightOperation.PRESET_SUBWOOFER),
				new IndexerShootCommand(indexerSubsystem),
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
				alignCommand,
				new AimHeightCommand(aimerSubsystem, targeter, null), // TODO Aim height
				pneumaticsSubsystem.downIntakeCommand(),
				CommandComposer.getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000), // TODO: Possibly
																									// change this value
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
			AimerSubsystem aimerSubsystem, Targeter targeter, PneumaticsSubsystem pneumaticsSubsystem,
			SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
			FlywheelSubsystem flywheelSubsystem) {
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreRightAutoRed(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note
				getTwoScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 30, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, 40, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoRed(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		if (visionSubsystem != null) {

			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		SequentialCommandGroup alignCommand = new SequentialCommandGroup(
				new TurnToAngleCommand(driveSubsystem, -20, 2, false));
		if (visionSubsystem != null) {
			alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem));
		}
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, aimerSubsystem, visionSubsystem,
						intakeSubsystem,
						indexerSubsystem, flywheelSubsystem),
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
	public static Command getAmpTwoAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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
	public static Command getAmpTwoAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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
	public static Command getShootAndAmp(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			AimerSubsystem aimerSubsystem, SimpleVisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem) {
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

	public static Command getAmpCommand(AimerSubsystem aimerSubsystem, Targeter targeter,
			FlywheelSubsystem flywheelSubsystem) {
		return sequence(
				new AimHeightCommand(aimerSubsystem, targeter, AimHeightOperation.SET_PRESET_DEFAULT),
				new FlywheelCommand(flywheelSubsystem,
						FlywheelOperation.SET_VELOCITY, 1500, 1500)); // 1300 1650

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
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
				intakeSubsystem.forwardIntakeCommand(),
				IndexerCommand.getFowardCommand(indexerSubsystem));
	}
}
