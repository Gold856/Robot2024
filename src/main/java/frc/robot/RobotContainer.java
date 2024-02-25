// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.drive.BangBangDriveDistanceCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.NinjaStarCommand;
import frc.robot.commands.drive.PolarDriveCommand;
import frc.robot.commands.drive.SetSteeringCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.flywheel.FlywheelCommand.Operation;
import frc.robot.commands.indexer.IndexerCommand;
import frc.robot.commands.indexer.IndexerShootCommand;
import frc.robot.commands.indexer.IndexerStopCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
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
	private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
	private final SimpleVisionSubsystem m_visionSubsystem = new SimpleVisionSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		m_autoSelector.addOption("Test Steering", SetSteeringCommand.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new TurnToAngleCommand(m_driveSubsystem, 90, 0.5, true));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters",
				new BangBangDriveDistanceCommand(m_driveSubsystem, 2, 0.01));
		m_autoSelector.addOption("Polar Drive Two Meters", new PolarDriveCommand(m_driveSubsystem, 2, 180));
		m_autoSelector.addOption("Right Two Score Blue",
				CommandComposer.getTwoScoreRightAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Two Score Red",
				CommandComposer.getTwoScoreRightAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Two Score Blue",
				CommandComposer.getTwoScoreLeftAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Two Score Red",
				CommandComposer.getTwoScoreLeftAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Three Score Blue",
				CommandComposer.getThreeScoreRightAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Three Score Red",
				CommandComposer.getThreeScoreRightAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Three Score Blue",
				CommandComposer.getThreeScoreLeftAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Three Score Red",
				CommandComposer.getThreeScoreLeftAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Four Score Blue",
				CommandComposer.getFourScoreRightAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Right Four Score Red",
				CommandComposer.getFourScoreRightAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Four Score Blue",
				CommandComposer.getFourScoreLeftAutoBlue(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Left Four Score Red",
				CommandComposer.getFourScoreLeftAutoRed(m_driveSubsystem, m_arduinoSubsystem, m_visionSubsystem));
		m_autoSelector.addOption("Get Blocks Auto",
				CommandComposer.getBlocksAuto(m_driveSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Get Amp Auto",
				CommandComposer.getAmpTwoAuto(m_driveSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Get Shoot and Amp Auto",
				CommandComposer.getShootAndAmp(m_driveSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Absolute to Zero", new TurnToAngleCommand(m_driveSubsystem, 0, 0.5, false));
		m_autoSelector.addOption("Intake With Sensor",
				CommandComposer.getIntakeWithSensorCommand(m_intakeSubsystem, m_indexerSubsystem, m_arduinoSubsystem));
		m_autoSelector.addOption("Intake With Sensor and Pneumatics",
				CommandComposer.getTeleopIntakeCommand(m_intakeSubsystem, m_pneumaticsSubsystem, m_indexerSubsystem,
						m_arduinoSubsystem));
		m_autoSelector.addOption("Ninja Star Command", new NinjaStarCommand(m_driveSubsystem, 2, -2, 45));
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

		// ------------------LED Controls------------------------------------
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
		m_driverController.button(Button.kX).onTrue(new DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		m_driverController.button(Button.kOptions).onTrue(m_driveSubsystem.resetHeadingCommand());

		// -------------------Flywheel Controls--------------------------------
		m_driverController.button(Button.kTriangle)
				.onTrue(new FlywheelCommand(m_flywheelSubsystem, Operation.SET_VELOCITY, 8000));

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

		// ------------------Amp Bar Controls, removal later-------------------
		m_operatorController.button(Button.kX).onTrue(m_pneumaticsSubsystem.toggleAmpBarCommand());
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}