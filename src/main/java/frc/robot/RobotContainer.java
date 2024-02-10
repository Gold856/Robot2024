// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.BangBangDriveDistance;
import frc.robot.commands.ClimberDriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.PIDTurnCommand;
import frc.robot.commands.SetSteering;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);
	private final CommandGenericHID m_operator = new CommandGenericHID(ControllerConstants.kOperatorControllerPort);
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ArduinoSubsystem m_ArduinoSubsystem = new ArduinoSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
		// Configure the button bindings
		m_autoSelector.addOption("Test Steering",
				SetSteering.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters", new BangBangDriveDistance(m_driveSubsystem, 2, 0.01));
		m_autoSelector.addOption("PID Drive 2 Meters",
				DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		m_autoSelector.addOption("Knock Over Blocks",
				CommandComposer.getBlocksAuto(m_driveSubsystem));

		// SmartDashboard.putData(m_autoSelector);
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		new Trigger(() -> DriverStation.getMatchTime() >= 20)
				.onTrue(m_ArduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_controller.getRawAxis(Axis.kLeftY),
				() -> m_controller.getRawAxis(Axis.kLeftX),
				() -> m_controller.getRawAxis(Axis.kRightX)));

		m_climberSubsystem.setDefaultCommand(new ClimberDriveCommand(m_climberSubsystem,
				() -> m_controller.getRawAxis(Axis.kLeftY),
				() -> m_controller.getRawAxis(Axis.kRightY),
				0.1));

		m_controller.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd());
		m_controller.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		m_controller.button(Button.kX).onTrue(new DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
