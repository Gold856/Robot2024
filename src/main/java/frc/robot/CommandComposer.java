package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

// TODO TEST ALL OF THIS AND SEE IF DRIVE CAN WORK IN PARALLEL
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
			LimeLightSubsystem limelightSubsystem, PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
				new BangBangDriveDistanceCommand(driveSubsystem, -2, 0.01),
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
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, Targeter targeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new PolarDriveCommand(driveSubsystem, 1., 90, 0.01));
		// if (simpleVisionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// simpleVisionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, poseEstimationSubsystem, simpleVisionSubsystem),

				parallel(
						new PolarDriveCommand(driveSubsystem, 1, 90, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, poseEstimationSubsystem, simpleVisionSubsystem),

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
	public static Command getTwoScoreRightAutoBlue(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

				new PolarDriveCommand(driveSubsystem, 0.75, 180, 0.01),
				parallel(
						new PolarDriveCommand(driveSubsystem, 1.2, 240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, -35, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

				parallel(
						new PolarDriveCommand(driveSubsystem, 1.4, 240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, -180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		// SequentialCommandGroup alignCommand = new SequentialCommandGroup(
		// new TurnToAngleCommand(driveSubsystem, 25, 2, false));
		// if (visionSubsystem != null) {
		// alignCommand.addCommands(new SimpleVisionAlignCommand(driveSubsystem,
		// visionSubsystem));
		// }
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
				parallel(
						new PolarDriveCommand(driveSubsystem, 1.2, -240, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.4, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
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
	public static Command getTwoScoreLeftAutoRed(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem,
			AimerSubsystem aimerSubsystem, RegressionTargeter regressionTargeter,
			IndexerSubsystem indexerSubsystem, FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem,
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
				parallel(
						new PolarDriveCommand(driveSubsystem, 0.75, -180, 0.01),
						pneumaticsSubsystem.downIntakeCommand()),
				new PolarDriveCommand(driveSubsystem, 1, -240, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, 2, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, 0.2, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
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
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				// right note
				getTwoScoreRightAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, 75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, 180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				// right note
				getTwoScoreLeftAutoBlue(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						targeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, targeter, limeLightSubsystem, poseEstimationSubsystem, simpleVisionSubsystem),
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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				// right note
				getTwoScoreLeftAutoRed(driveSubsystem, arduinoSubsystem, simpleVisionSubsystem, aimerSubsystem,
						regressionTargeter,
						indexerSubsystem, flywheelSubsystem, intakeSubsystem,
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// middle note
				new TurnToAngleCommand(driveSubsystem, -75, 2, false),
				new PolarDriveCommand(driveSubsystem, 1.25, -180, 0.01),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
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
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 2, 310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				new PolarDriveCommand(driveSubsystem, .5, 180, 0.01),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
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
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// left note
				new PolarDriveCommand(driveSubsystem, 1.5, 90),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, 0),
				parallel(
						new PolarDriveCommand(driveSubsystem, -0.2, 180), // -180
						intakeSubsystem.forwardIntakeCommand()),
				// alignCommand,
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),
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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
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
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 2, -310, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .45, -180, 0.01),
				parallel(
						new PolarDriveCommand(driveSubsystem, -0.2, 180), // -180
						intakeSubsystem.forwardIntakeCommand()),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			PneumaticsSubsystem pneumaticsSubsystem, LimeLightSubsystem limeLightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
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
						pneumaticsSubsystem, limeLightSubsystem, poseEstimationSubsystem),
				// right note
				new PolarDriveCommand(driveSubsystem, 1.8, -280, 0.01),
				new TurnToAngleCommand(driveSubsystem, 0, false),
				new PolarDriveCommand(driveSubsystem, .35, 180, 0.01), // -180
				intakeSubsystem.forwardIntakeCommand(),
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limeLightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
				new TimedLEDCommand(arduinoSubsystem, 0.25, StatusCode.RAINBOW_PARTY_FUN_TIME),
				// strafe away, intake
				new PolarDriveCommand(driveSubsystem, 1, -90),
				new PolarDriveCommand(driveSubsystem, 0.4, 180),
				getIntakeWithSensorCommand(intakeSubsystem, indexerSubsystem, arduinoSubsystem),
				// go back and shoot
				new PolarDriveCommand(driveSubsystem, 1, 90),
				new PolarDriveCommand(driveSubsystem, 0.4, 0),
				getAmpCommand(aimerSubsystem, regressionTargeter, flywheelSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel(),
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
			SimpleVisionSubsystem simpleVisionSubsystem, LimeLightSubsystem limelightSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, poseEstimationSubsystem,
						simpleVisionSubsystem),

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
			ArduinoSubsystem arduinoSubsystem) { // TODO USE THIS
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
			LimeLightSubsystem limelightSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		return sequence(
				new AimHeightCommand(aimerSubsystem, regressionTargeter, AimHeightOperation.SET_PRESET_DEFAULT),
				parallel(
						sequence(
								new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000, 8000),
								new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SETTLE, 0, 0)),
						new AimHeightCommand(aimerSubsystem, regressionTargeter,
								AimHeightOperation.CALC_AND_SET, limelightSubsystem, poseEstimationSubsystem),
						// new SimpleVisionAlignCommand(driveSubsystem, visionSubsystem)),
						getTurnToClosestSpeakerCommand(driveSubsystem, limelightSubsystem)));
		// new IndexerShootCommand(indexerSubsystem),
		// flywheelSubsystem.stopFlywheel());
	}

	public static Command getAimAndShootAutoCommand(DriveSubsystem driveSubsystem,
			SimpleVisionSubsystem simpleVisionSubsystem, FlywheelSubsystem flywheelSubsystem,
			AimerSubsystem aimerSubsystem, IndexerSubsystem indexerSubsystem, Targeter regressionTargeter,
			LimeLightSubsystem limelightSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			SimpleVisionSubsystem visionSubsystem) {
		return sequence(
				getAimAndShootAutoCommand(driveSubsystem, simpleVisionSubsystem, flywheelSubsystem, aimerSubsystem,
						indexerSubsystem, regressionTargeter, limelightSubsystem, poseEstimationSubsystem,
						visionSubsystem),
				new IndexerShootCommand(indexerSubsystem),
				flywheelSubsystem.stopFlywheel());
	}

	public static Command getSourcePickUpCommand(AimerSubsystem aimerSubsystem, Targeter targeter,
			FlywheelSubsystem flywheelSubsystem, IndexerSubsystem indexerSubsystem) {
		return sequence(
				parallel(
						new AimHeightCommand(aimerSubsystem, targeter, AimHeightOperation.SOURCE),
						new FlywheelCommand(flywheelSubsystem, FlywheelOperation.SET_VELOCITY, -2000, -2000),
						IndexerCommand.getReverseCommand(indexerSubsystem)));
	}

	public static Command getStopFlywheelAndIndexer(FlywheelSubsystem flywheelSubsystem,
			IndexerSubsystem indexerSubsystem) {
		return sequence(
				parallel(
						new IndexerStopCommand(indexerSubsystem),
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
}