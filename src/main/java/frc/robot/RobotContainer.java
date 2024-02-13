// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.drive.BangBangDriveDistance;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.PIDTurnCommand;
import frc.robot.commands.drive.SetSteering;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

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
	private final ArduinoSubsystem m_ArduinoSubsystem = new ArduinoSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
		// Configure the button bindings
		m_autoSelector.addOption("Test Steering", SetSteering.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters", new BangBangDriveDistance(m_driveSubsystem, 2, 0.01));
		m_autoSelector.addOption("PID Drive 2 Meters", DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		m_autoSelector.addOption("Knock Over Blocks",
				CommandComposer.getBlocksAuto(m_driveSubsystem));

		SmartDashboard.putData(m_autoSelector);
		configureButtonBindings();
	}

	static class Pose extends Pose2d {

		Pose(double x, double y, double yawInDegrees) {
			super(x, y, Rotation2d.fromDegrees(yawInDegrees));
		}

	};

	static class DriveCommandSample extends DriveCommand { // more code for debugging purposes

		protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

		public DriveCommandSample(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
				double angleTolerance) {
			super(driveSubsystem, targetPose, distanceTolerance, angleTolerance);
		}

		@Override
		public void recordPose(String entryName, Pose2d value) {
			if (value == null)
				table.getEntry(entryName).setDoubleArray(new double[0]);
			else
				table.getEntry(entryName).setDoubleArray(toPose2DAdvantageScope(value.getX(),
						value.getY(), value.getRotation().getDegrees()));
		}

		@Override
		public void recordString(String entryName, String value) {
			table.getEntry(entryName).setString(value);
		}

		static double[] toPose2DAdvantageScope(double x, double y, double yawInDegrees) {
			return new double[] { x + 8.27, y + 4.1, yawInDegrees * Math.PI / 180 };
		}

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// new Trigger(() -> DriverStation.getMatchTime() >= 20)
		// .onTrue(m_ArduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_driverController.getRawAxis(Axis.kLeftY),
				() -> m_driverController.getRawAxis(Axis.kLeftX),
				() -> m_driverController.getRawAxis(Axis.kRightTrigger),
				() -> m_driverController.getRawAxis(Axis.kLeftTrigger)

		));

		m_driverController.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		m_driverController.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd());
		m_driverController.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		Command[] samples = { new WaitCommand(0.2) {
			@Override
			public void initialize() {
				super.initialize();
				m_driveSubsystem.setModuleStates(0.1, 0, 0, false);
			}

			@Override
			public void execute() {
				super.execute();
				m_driveSubsystem.setModuleStates(0.0, 0.1, 0, false);
			}

			@Override
			public void end(boolean b) {
				super.end(b);
				m_driveSubsystem.setModuleStates(0.0, 0.1, 0, false);
			}
		}, new DriveCommandSample(m_driveSubsystem, new Pose(0, 0, 30), 0.05, 1)
				.andThen(new DriveCommandSample(m_driveSubsystem, new Pose(0, 0, 0), 0.05,
						1)) };
		m_driverController.button(Button.kX).whileTrue(samples[0]);

		// // Indexer Button Mappings
		// m_operatorController.button(Button.kSquare).onTrue(new
		// IndexerFowardCommand(0.5)); // TODO add constants
		// m_operatorController.button(Button.kCircle).onTrue(new
		// IndexerReverseCommand(0.5));
		// m_operatorController.button(Button.kTrackpad).onTrue(new
		// IndexerStopCommand());
		// m_operatorController.button(Button.kRightBumper).onTrue(new
		// IndexerShootCommand(.5, .5));

	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
