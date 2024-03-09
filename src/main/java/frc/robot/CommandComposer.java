package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeter.RegressionTargeter;
import frc.robot.commands.TimedLEDCommand;
import frc.robot.commands.aimshooter.AimHeightCommand;
import frc.robot.commands.aimshooter.AimHeightCommand.AimHeightOperation;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexWithSensorCommand;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.subsystems.AimerSubsystem;
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

// TODO UPDATE DOCUMENTATION
// TODO All the autos (update so it is not using the AimAndShoot teleop one)

public class CommandComposer {
	/**
	 * Returns a command to shoot a note and leave the wing (end with LEDS).
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getShootAndLeaveAuto(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem,
			LimeLightSubsystem limelightSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, arduinoSubsystem),
				new BangBangDriveDistanceCommand(driveSubsystem, -2, 0.01),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getBlueShootAndLeaveAuto(DriveSubsystem driveSubsystem,
			ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem,
			LimeLightSubsystem limelightSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem,
						flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-6.0 + 0.55, 0.0 - 3.0, 180 - 25),
						0.1, 5, driveSubsystem,
						limelightSubsystem),
				new TimedLEDCommand(arduinoSubsystem, 0.25,
						StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getRedShootAndLeaveAuto(DriveSubsystem driveSubsystem,
			ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem,
			LimeLightSubsystem limelightSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem,
						flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(6.0 - 0.55, 0.0 - 3.0, 25),
						0.1, 5, driveSubsystem,
						limelightSubsystem),
				new TimedLEDCommand(arduinoSubsystem, 0.25,
						StatusCode.RAINBOW_PARTY_FUN_TIME));
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, Targeter targeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new PolarDriveCommand(driveSubsystem, 1., 90, 0.01));
		// if (simpleVisionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// simpleVisionSubsystem));
		// }
		return sequence(
				parallel(
						pneumaticsSubsystem.downIntakeCommand(),
						getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem,
								aimerSubsystem,
								indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem)),
				race(
						getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
						new PolarDriveCommand(driveSubsystem, 1, 180, 0.07)),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem,
						aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				new PolarDriveCommand(driveSubsystem, 0.75, 180, 0.01),
				parallel(
						new PolarDriveCommand(driveSubsystem, 1.2, 240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				parallel(
						new PolarDriveCommand(driveSubsystem, 1.4, 240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, 25, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getSubwooferShotAutoCommand(flywheelSubsystem, aimerSubsystem, indexerSubsystem),
				parallel(
						new PolarDriveCommand(driveSubsystem, 1.2, -240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),
				// alignCommand,
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	public static Command getSubwooferShotAutoCommand(FlywheelSubsystem flywheelSubsystem,
			AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem) {
		return sequence(
				parallel(
						new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 4000, 4000),
						sequence(new AimHeightCommand(aimerSubsystem, null, AimHeightOperation.PRESET_SUBWOOFER))),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel());
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return sequence(
				getSubwooferShotAutoCommand(flywheelSubsystem, aimerSubsystem, indexerSubsystem),
				parallel(
						new PolarDriveCommand(driveSubsystem, 0.75, -180, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new PolarDriveCommand(driveSubsystem, 1, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, 0, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				// right note
				getTwoScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return sequence(
				// right note
				getTwoScoreRightAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter targeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return sequence(
				// right note
				getTwoScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						targeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		return sequence(
				// right note
				getTwoScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, 30, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoBlue(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 2, 310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, .5, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, 40, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				// right note and middle note
				getThreeScoreRightAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 1.5, 90),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, 0),
				parallel(
						new PolarDriveCommand(driveSubsystem, -0.2, 180), // -180
						intakeSubsystem.forwardIntakeCommand()),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 2, -310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, -180, 0.01),
				parallel(
						new PolarDriveCommand(driveSubsystem, -0.2, 180), // -180
						intakeSubsystem.forwardIntakeCommand()),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -20, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				// left note and middle note
				getThreeScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 1.8, -280, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .35, 180, 0.01), // -180
				intakeSubsystem.forwardIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, arduinoSubsystem),

				// alignCommand,
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
			IndexerSubsystem indexerSubsystem, PneumaticsSubsystem pneumaticsSubsystem, AimerSubsystem aimerSubsystem,
			RegressionTargeter regressionTargeter, FlywheelSubsystem flywheelSubsystem,
			IntakeSubsystem intakeSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				pneumaticsSubsystem.downIntakeCommand(),
				new PolarDriveCommand(driveSubsystem, 0.22, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel(),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME));
		// strafe away, intake
		// new PolarDriveCommand(driveSubsystem, 1.0, 0.2, -90, 5.0));
		// new PolarDriveCommand(driveSubsystem, 0.4, 180),
		// getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem,
		// arduinoSubsystem),
		// // go back and shoot
		// new PolarDriveCommand(driveSubsystem, 1, 90),
		// new PolarDriveCommand(driveSubsystem, 0.4, 0),
		// getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
		// new IndexerShootCommand(indexerSubsystem),
		// flywheelSubsystem.stopFlywheel(),
		// new TimedLEDCommand(arduinoSubsystem, 0.4,
		// StatusCode.RAINBOW_PARTY_FUN_TIME));
	}

	/**
	 * 
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param arduinoSubsystem The arduino subsystem for LEDs.
	 * @return The command.
	 */
	public static Command getAmpTwoAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			IndexerSubsystem indexerSubsystem, PneumaticsSubsystem pneumaticsSubsystem, AimerSubsystem aimerSubsystem,
			RegressionTargeter regressionTargeter, FlywheelSubsystem flywheelSubsystem,
			IntakeSubsystem intakeSubsystem) {
		return sequence(
				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 0.22, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel(),
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel(),
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
			IndexerSubsystem indexerSubsystem, PneumaticsSubsystem pneumaticsSubsystem, AimerSubsystem aimerSubsystem,
			RegressionTargeter regressionTargeter, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem, LimeLightSubsystem limelightSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, arduinoSubsystem),

				// strafe towards speaker, shoot
				new PolarDriveCommand(driveSubsystem, 1.4, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// strafe away, intake
				new TurnToAngleCommand(driveSubsystem, -90, 2, false),
				new PolarDriveCommand(driveSubsystem, 0.70, 90),
				new PolarDriveCommand(driveSubsystem, -1.05, 180),
				getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel());
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

	public static Command getAmpCommand(AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			FlywheelSubsystem flywheelSubsystem) {
		return sequence(
				new AimHeightCommand(aimerSubsystem, regressionTargeter, AimHeightOperation.SET_PRESET_DEFAULT),
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

	public static Command getAimAndShootCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem, FlywheelSubsystem flywheelSubsystem,
			AimerSubsystem aimerSubsystem, IndexerSubsystem indexerSubsystem, Targeter regressionTargeter,
			LimeLightSubsystem limelightSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				new AimHeightCommand(aimerSubsystem, regressionTargeter, AimHeightOperation.SET_PRESET_DEFAULT),
				parallel(
						sequence(
								new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
								new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SETTLE, 0, 0)),
						new AimHeightCommand(aimerSubsystem, regressionTargeter,
								AimHeightOperation.CALC_AND_SET, limelightSubsystem)
										.andThen(arduinoSubsystem.writeStatus(StatusCode.SOLID_BLUE)),
						// new SimpleVisionAlignCommand(driveSubsystem, simpleVisionSubsystem)),
						getTurnToClosestSpeakerCommand(driveSubsystem, limelightSubsystem)));
		// new IndexerShootCommand(indexerSubsystem),
		// flywheelSubsystem.stopFlywheel());
	}

	public static Command getAimAndShootAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem, FlywheelSubsystem flywheelSubsystem,
			AimerSubsystem aimerSubsystem, IndexerSubsystem indexerSubsystem, Targeter regressionTargeter,
			LimeLightSubsystem limelightSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, arduinoSubsystem).withTimeout(2),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel());
	}

	public static Command getSourcePickUpCommand(AimerSubsystem aimerSubsystem, Targeter targeter,
			FlywheelSubsystem flywheelSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
		return parallel(
				new AimHeightCommand(aimerSubsystem, targeter, AimHeightOperation.SOURCE),
				new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, -2000, -2000));
		// IndexerCommand.getReverseCommand(indexerSubsystem),
		// intakeSubsystem.reverseIntakeCommand()

	}

	public static Command getStopFlywheelAndIndexer(FlywheelSubsystem flywheelSubsystem,
			IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
		return sequence(
				parallel(
						new IndexerStopCommand(indexerSubsystem),
						intakeSubsystem.stopIntakeCommand(),
						flywheelSubsystem.stopFlywheel()));
	}

	public static Command getTurnToClosestSpeakerCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var target = limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = limeLightSubsystem.transformationToward(target);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5);
	}

	public static Command getMoveTowardClosestSpeakerCommand(double distanceToTarget, DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s = () -> {
			var target = limeLightSubsystem.closest(kBlueSpeakerPosition, kRedSpeakerPosition);
			var t = limeLightSubsystem.transformationToward(target, distanceToTarget);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s, 0.1, 5);
	}

	public static Command getAlignToClosestAmpCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem) {
		Supplier<Pose2d> s1 = () -> {
			Pose target = new Pose(limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose));
			var t = limeLightSubsystem.transformationTo(target.add(new Pose(0, -0.3, 0)));
			return driveSubsystem.getPose().plus(t);
		};
		Supplier<Pose2d> s2 = () -> {
			var target = limeLightSubsystem.closest(kBlueAmpPose, kRedAmpPose);
			var t = limeLightSubsystem.transformationTo(target);
			return driveSubsystem.getPose().plus(t);
		};
		return new DriveCommand(driveSubsystem, s1, 0.1, 5).andThen(new DriveCommand(driveSubsystem, s2, 0.1, 5));
	}

	public static Command getShootToClosestSpeakerAtCommand(Pose2d targetPose, double timeout,
			DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return DriveCommand.alignTo(targetPose, 0.1, 5, driveSubsystem,
				limeLightSubsystem).withTimeout(timeout).andThen(
						getAimAndShootAutoCommand(driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
								targeter, limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getPickUpNoteAtCommand(Pose2d targetPose, DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return DriveCommand
				// TODO: originally .3
				.alignTo(targetPose.plus(new Transform2d(0.6, 0, Rotation2d.fromDegrees(0))), 0.2, 5, driveSubsystem,
						limeLightSubsystem)
				.andThen(parallel(
						DriveCommand.alignTo(targetPose, 0.1, 5, driveSubsystem,
								limeLightSubsystem),
						getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem,
								arduinoSubsystem)));
	}

	public static Command getPickUpNoteAndShootAtCommand(Pose2d targetPose, double timeout,
			DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getPickUpNoteAtCommand(targetPose, driveSubsystem,
				visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
				targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						.withTimeout(timeout).andThen(
								getAimAndShootAutoCommand(driveSubsystem,
										visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
										targeter, limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getFourScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return pneumaticsSubsystem.downIntakeCommand()
				.andThen(
						getShootToClosestSpeakerAtCommand(kBlueNoteThreePose.add(new Pose(-0.65, 0, 0)), 1.5,
								driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, arduinoSubsystem))
				.andThen( // 2nd note
						getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				.andThen( // 3rd note
						getPickUpNoteAndShootAtCommand(kBlueNoteTwoPose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				.andThen( // 4th note
						getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem));
	}

	public static Command getFourScoreRedAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return pneumaticsSubsystem.downIntakeCommand()
				.andThen(
						getShootToClosestSpeakerAtCommand(kRedNoteThreePose.add(new Pose(0.6, 0, 0)), 1.5,
								driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem,
								arduinoSubsystem))
				.andThen( // 2nd note
						getPickUpNoteAndShootAtCommand(kRedNoteThreePose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				.andThen( // 3rd note
						getPickUpNoteAndShootAtCommand(kRedNoteTwoPose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				.andThen( // 4th note
						getPickUpNoteAndShootAtCommand(kRedNoteOnePose, 2.5, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem));
	}

	public static Command getFiveScoreBlueAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getFourScoreBlueAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						.andThen(getPickUpNoteAtCommand(kBlueCenterNoteOnePose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
						.andThen(getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getFiveScoreRedAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getFourScoreRedAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						.andThen(getPickUpNoteAtCommand(kRedCenterNoteOnePose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
						.andThen(getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getTwoMiddleFourScoreRedCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				getPickUpNoteAndShootAtCommand(kRedNoteThreePose, 5, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem)

		);
	}

	public static Command getTwoMiddleFourScoreBlueCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				getPickUpNoteAndShootAtCommand(kBlueNoteThreePose, 5, driveSubsystem, visionSubsystem,
						flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem)

		);
	}

	public static Command getThreeMiddleFourScoreBlueCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.moveToward(new Translation2d(-6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 155), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-1, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteThreePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(-1, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(-6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(-6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem)

		);
	}

	public static Command getThreeMiddleFourScoreRedCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				pneumaticsSubsystem.downIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.moveToward(new Translation2d(6, -2), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(2, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(1, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteThreePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(2),
				DriveCommand.moveToward(new Translation2d(1, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				DriveCommand.moveToward(new Translation2d(6, -3), 0, 0.5, 360, driveSubsystem, limeLightSubsystem),
				getShootToClosestSpeakerAtCommand(new Pose(6, -0.5, 25), 3, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						arduinoSubsystem)

		);
	}

	public static Command getThreeScoreTwoMiddleBottomBlueAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				pneumaticsSubsystem.downIntakeCommand(),
				getPickUpNoteAtCommand(kBlueCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 0.4, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kBlueCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.alignTo(new Pose(-5.5, -3, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(-6.25, 0.4, 180 + 25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getThreeScoreTwoMiddleBottomRedAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				pneumaticsSubsystem.downIntakeCommand(),
				getPickUpNoteAtCommand(kRedCenterNoteFourPose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 0.4, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getPickUpNoteAtCommand(kRedCenterNoteFivePose, driveSubsystem, visionSubsystem, flywheelSubsystem,
						aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem,
						pneumaticsSubsystem, arduinoSubsystem).withTimeout(4),
				DriveCommand.alignTo(new Pose(5.5, -3, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				DriveCommand.alignTo(new Pose(6.25, 0.4, -25), 0.3, 10, driveSubsystem, limeLightSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getThreeScoreOneMiddleTopBlueAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return pneumaticsSubsystem.downIntakeCommand()
				.andThen(
						getShootToClosestSpeakerAtCommand(kBlueNoteOnePose.add(new Pose(-0.4, -0.4, 0)), 1.5,
								driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem,
								targeter,
								limeLightSubsystem,
								arduinoSubsystem))
				// 2nd note
				.andThen(getPickUpNoteAndShootAtCommand(kBlueNoteOnePose, 1.5, driveSubsystem,
						visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
						limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				// 3rd note
				.andThen(getPickUpNoteAtCommand(kBlueCenterNoteOnePose, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
				.andThen(getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4, driveSubsystem,
						visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
						limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getThreeScoreOneMiddleTopRedAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return pneumaticsSubsystem.downIntakeCommand()
				.andThen(getShootToClosestSpeakerAtCommand(kRedNoteOnePose.add(new Pose(0.4, -0.4, 0)), 1.5,
						driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
						limeLightSubsystem,
						arduinoSubsystem))
				// 2nd note
				.andThen(getPickUpNoteAndShootAtCommand(kRedNoteOnePose, 1.5, driveSubsystem,
						visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
						limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem))
				// 3rd note
				.andThen(getPickUpNoteAtCommand(kRedCenterNoteOnePose, driveSubsystem, visionSubsystem,
						flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
						intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
				.andThen(getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4, driveSubsystem,
						visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
						limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getFourScoreTwoMiddleTopBlueAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getThreeScoreOneMiddleTopBlueAuto(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						// 4th note
						.andThen(getPickUpNoteAtCommand(kBlueCenterNoteTwoPose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
						.andThen(getShootToClosestSpeakerAtCommand(kBlueNoteOnePose, 4, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, arduinoSubsystem));
	}

	public static Command getFourScoreTwoMiddleTopRedAuto(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem visionSubsystem, FlywheelSubsystem flywheelSubsystem, AimerSubsystem aimerSubsystem,
			IndexerSubsystem indexerSubsystem, Targeter targeter,
			LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return getThreeScoreOneMiddleTopRedAuto(driveSubsystem, visionSubsystem, flywheelSubsystem, aimerSubsystem,
				indexerSubsystem, targeter, limeLightSubsystem, intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem)
						// 4th note
						.andThen(getPickUpNoteAtCommand(kRedCenterNoteTwoPose, driveSubsystem, visionSubsystem,
								flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter, limeLightSubsystem,
								intakeSubsystem, pneumaticsSubsystem, arduinoSubsystem).withTimeout(4))
						.andThen(getShootToClosestSpeakerAtCommand(kRedNoteOnePose, 4, driveSubsystem,
								visionSubsystem, flywheelSubsystem, aimerSubsystem, indexerSubsystem, targeter,
								limeLightSubsystem, arduinoSubsystem));
	}
}
