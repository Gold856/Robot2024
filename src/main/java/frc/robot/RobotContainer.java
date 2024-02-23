// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.climber.ClimberDriveCommand;
import frc.robot.commands.climber.ClimberPresetCommand;
import frc.robot.commands.climber.ClimberPresetCommand.ClimberOperation;
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.SetSteeringCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.FlywheelOperation;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SimpleVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final CommandGenericHID m_driverController = new CommandGenericHID(
			ControllerConstants.kDriverControllerPort);
	private final CommandGenericHID m_operatorController = new CommandGenericHID(
			ControllerConstants.kOperatorControllerPort);
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
	private final SimpleVisionSubsystem m_visionSubsystem = new SimpleVisionSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final LimeLightSubsystem m_limeLightSubsystem = new PoseEstimationSubsystem() {
		{
			addPoseSupplier("BotPose@Odometry", () -> m_driveSubsystem.getPose());
		}
	};

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		m_autoSelector.addOption("Test Steering", SetSteeringCommand.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new TurnToAngleCommand(m_driveSubsystem, 90.0, 0.5, true));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters",
				new BangBangDriveDistanceCommand(m_driveSubsystem, 2, 0.01));
		m_autoSelector.addOption("PID Drive 2 Meters", DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		m_autoSelector.addOption("Right Two Score",
				CommandComposer.getTwoScoreRightAuto(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Two Score",
				CommandComposer.getTwoScoreLeftAuto(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Three Score",
				CommandComposer.getThreeScoreRightAuto(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Three Score",
				CommandComposer.getThreeScoreLeftAuto(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Get Blocks Auto",
				CommandComposer.getBlocksAuto(m_driveSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Absolute to Zero", new TurnToAngleCommand(m_driveSubsystem, 0, 0.5, false));
		m_autoSelector.addOption("Intake With Sensor",
				CommandComposer.getIntakeWithSensorCommand(m_intakeSubsystem, m_indexerSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Intake With Sensor and Pneumatics",
				CommandComposer.getTeleopIntakeCommand(m_intakeSubsystem, m_pneumaticsSubsystem, m_indexerSubsystem,
						m_arduinoSubsystem));

		SmartDashboard.putData(m_autoSelector);
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */

	private void configureButtonBindings() {

		// --------------------- LED Controls ---------------------------
		// Should have RainbowPartyFunTime in the last 20 seconds of a match
		new Trigger(() -> DriverStation.getMatchTime() <= 20)
				.onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		// LEDs for when you want AMP
		m_operatorController.povLeft().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_PURPLE));
		// LEDs for when you want CO-OP
		m_operatorController.povUp().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_YELLOW));
		// LEDs for when you want HP to drop a note
		m_operatorController.povRight().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.BLINKING_RED));
		// DEFAULT Button
		m_operatorController.povDown().onTrue(m_arduinoSubsystem.writeStatus(StatusCode.DEFAULT));
		// RainbowPartyFunTime
		m_operatorController.button(Button.kShare)
				.onTrue(m_arduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));

		// --------------------Drive Controls---------------------------------
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_driverController.getRawAxis(Axis.kLeftY),
				() -> m_driverController.getRawAxis(Axis.kLeftX),
				() -> m_driverController.getRawAxis(Axis.kRightTrigger),
				() -> m_driverController.getRawAxis(Axis.kLeftTrigger)));
		m_driverController.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		m_driverController.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		m_driverController.button(Button.kX).onTrue(new DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		m_driverController.button(Button.kOptions).onTrue(m_driveSubsystem.resetHeadingCommand());

		// -------------------Flywheel Controls--------------------------------
		m_driverController.button(Button.kTriangle)
				.onTrue(new FlywheelCommand(m_flywheelSubsystem, FlywheelOperation.SET_VELOCITY, 8000));

		// -------------------Indexer Controls---------------------------------
		m_driverController.button(Button.kCircle).onTrue(new IndexerShootCommand(m_indexerSubsystem));
		m_operatorController.button(Button.kRightBumper).onTrue(new IndexerShootCommand(m_indexerSubsystem));

		// ------------------Intake Controls-----------------------------------
		m_operatorController.button(Button.kLeftTrigger).onTrue(CommandComposer.getTeleopIntakeCommand(
				m_intakeSubsystem, m_pneumaticsSubsystem, m_indexerSubsystem, m_arduinoSubsystem));
		m_operatorController.button(Button.kRightTrigger)
				.onTrue(m_pneumaticsSubsystem.upIntakeCommand().andThen(m_intakeSubsystem.stopIntakeCommand()));
		// sorry about this one
		m_operatorController.povLeft().and(m_operatorController.button(Button.kLeftBumper)).onTrue(m_intakeSubsystem
				.reverseIntakeCommand().alongWith(IndexerCommand.getReverseCommand(m_indexerSubsystem)));
		// and this one
		m_operatorController.povLeft().and(m_operatorController.button(Button.kLeftBumper))
				.onFalse(m_intakeSubsystem.stopIntakeCommand().alongWith(new IndexerStopCommand(m_indexerSubsystem)));

		// ------------------Amp Bar Controls, removal later------------------- TODO
		m_operatorController.button(Button.kX).onTrue(m_pneumaticsSubsystem.toggleAmpBarCommand());

		// -------------------Climber Commands---------------------------------
		m_climberSubsystem.setDefaultCommand(new ClimberDriveCommand(m_climberSubsystem,
				() -> m_operatorController.getRawAxis(Axis.kLeftY),
				() -> m_operatorController.getRawAxis(Axis.kRightY)));

		m_operatorController.button(Button.kTriangle)
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.TOP,
						() -> m_operatorController.getRawAxis(Axis.kLeftY),
						() -> m_operatorController.getRawAxis(Axis.kRightY)));
		m_operatorController.button(Button.kX)
				.onTrue(new ClimberPresetCommand(m_climberSubsystem, ClimberOperation.ZERO,
						() -> m_operatorController.getRawAxis(Axis.kLeftY),
						() -> m_operatorController.getRawAxis(Axis.kRightY)));

		Command[] samples = {
				DriveCommand.towardRedSpeaker(m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.toRedAmp(m_driveSubsystem, m_limeLightSubsystem),
				DriveCommand.toward(new Translation2d(7.87, 1.45),
						1.2, 0.1, 5, m_driveSubsystem, m_limeLightSubsystem)
						.andThen(
								DriveCommand.to(new Pose(6.5, 0.0, 0), 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(
								DriveCommand.to(new Pose(6.0, 0.0, 0), 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(DriveCommand.toward(new Translation2d(7.87, 1.45),
								1.2, 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(
								DriveCommand.to(new Pose(6.5, 1.45, 0), 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(
								DriveCommand.to(new Pose(6.0, 1.45, 0), 0.1, 5, m_driveSubsystem,
										m_limeLightSubsystem))
						.andThen(DriveCommand.toward(new Translation2d(7.87, 1.45),
								1.2, 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(
								DriveCommand.to(new Pose(6.5, 2.92, 0), 0.1, 5, m_driveSubsystem, m_limeLightSubsystem))
						.andThen(
								DriveCommand.to(new Pose(6.0, 2.82, 0), 0.1, 5, m_driveSubsystem,
										m_limeLightSubsystem)),
				new DriveCommand(m_driveSubsystem, 1, 1, 60, 0.1, 5),
				new DriveCommand(m_driveSubsystem, new Pose2d(1.0, 0, Rotation2d.fromDegrees(0)), 0.1, 5)
						.andThen(
								new DriveCommand(m_driveSubsystem, new Pose2d(1, 1, Rotation2d.fromDegrees(0)), 0.1, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose2d(1, 1, Rotation2d.fromDegrees(45)), 0.1,
								5))
						.andThen(
								new DriveCommand(m_driveSubsystem, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 0.1, 5))
		};
		m_driverController.button(Button.kShare)
				.whileTrue(samples[0]);
		m_driverController.button(Button.kTrackpad)
				.whileTrue(samples[1]);
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}