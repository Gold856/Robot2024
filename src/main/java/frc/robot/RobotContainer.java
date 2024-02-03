// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.common.PoseEstimationSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.BangBangDriveDistance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.PIDTurnCommand;
import frc.robot.commands.SetSteering;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements frc.common.RobotContainer {
	private final CommandGenericHID m_controller = new CommandGenericHID(ControllerConstants.kDriverControllerPort);
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem();
	private final ArduinoSubsystem m_ArduinoSubsystem = new ArduinoSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// m_poseEstimationSubsystem.addPoseSupplier("Pose2D@Odometry@SwerveBot",
		// () -> m_driveSubsystem.getPose());

		// Configure the button bindings
		m_autoSelector.addOption("Drive 2 Meters", new DriveDistanceCommand(m_driveSubsystem, 2, .1));
		m_autoSelector.addOption("Test Steering", SetSteering.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters", new BangBangDriveDistance(m_driveSubsystem, 2));
		m_autoSelector.addOption("PID Drive 2 Meters", DriveDistanceCommand.create(m_driveSubsystem, 2.0));
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
		new Trigger(() -> DriverStation.getMatchTime() >= 20)
				.onTrue(m_ArduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_controller.getRawAxis(Axis.kLeftY),
				() -> m_controller.getRawAxis(Axis.kLeftX),
				() -> m_controller.getRawAxis(Axis.kRightX)));
		m_controller.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd());
		m_controller.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		m_controller.button(Button.kX)
				.whileTrue(DriveCommand.createCommand(0.05, 1,
						new Pose2d(6.85, 3.0, Rotation2d.fromDegrees(0)),
						new Pose2d(6, 3.0, Rotation2d.fromDegrees(0)),
						new Pose2d(6.44, 3.5, Rotation2d.fromDegrees(90)),
						new Pose2d(6.44, 3.7, Rotation2d.fromDegrees(90)),
						new Pose2d(4.2, 1.5, Rotation2d.fromDegrees(-120)),
						new Pose2d(6.85, 3.0, Rotation2d.fromDegrees(0))));
		new Command() { // sample
			public void initialize() {
				var pose = m_poseEstimationSubsystem.estimatedPose();
				System.out.println("estimated pose: " + pose);
			}
		};
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
