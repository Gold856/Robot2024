// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.commands.BangBangDriveDistance;
import frc.robot.commands.DriveCommandAdvanced;
import frc.robot.commands.PIDTurnCommand;
import frc.robot.commands.SetSteering;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightEmulationSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.Pose;
import frc.robot.subsystems.PoseEstimationSubsystem;

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
	protected final ArduinoSubsystem m_ArduinoSubsystem = new ArduinoSubsystem();
	private final SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();
	// private final LimeLightSubsystem m_limeLightSubsystem = new
	// LimeLightSubsystem() {
	private final LimeLightSubsystem m_limeLightSubsystem = new PoseEstimationSubsystem() {
		{
			addPoseSupplier("BotPose@Odometry", () -> m_driveSubsystem.getPose());
		}

		protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

		@Override
		public void periodic() {
			super.periodic();
			var pose = estimatedPose();
			table.getEntry("Pose Estimated").setString("" + pose);
			if (pose != null)
				table.getEntry("BotPose'").setDoubleArray(toPose2DAdvantageScope(pose));
			pose = m_driveSubsystem.getPose();
			table.getEntry("BotPose@Odometry").setDoubleArray(toPose2DAdvantageScope(pose));
			try {
				pose = new Pose(m_botpose.value[0], m_botpose.value[1], m_botpose.value[5]);
				table.getEntry("BotPose").setDoubleArray(toPose2DAdvantageScope(pose));
			} catch (Exception e) {
			}
		}

		@Override
		public void recordPose(String entryName, Pose2d value) {
			if (value != null && !(value instanceof Pose))
				value = new Pose(value.getX(), value.getY(), value.getRotation().getDegrees());
			if (value == null)
				table.getEntry(entryName).setDoubleArray(new double[0]);
			else
				table.getEntry(entryName).setDoubleArray(toPose2DAdvantageScope(value));
		}

		@Override
		public void recordString(String entryName, String value) {
			table.getEntry(entryName).setString(value);
		}

		static double[] toPose2DAdvantageScope(Pose2d pose) {
			return pose == null ? new double[0]
					: new double[] { pose.getX() + 8.27, pose.getY() + 4.1,
							pose.getRotation().getDegrees() * Math.PI / 180 };
		}

	};

	static class TurnCommand extends frc.robot.commands.drive.TurnCommand { // more code for debugging purposes

		protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

		public TurnCommand(DriveSubsystem driveSubsystem, double targetAlgnle, double angleTolerance) {
			super(driveSubsystem, targetAlgnle, angleTolerance);
		}

		public TurnCommand(DriveSubsystem driveSubsystem, Translation2d targetPosition,
				LimeLightSubsystem limeLieghLightSubsystem, double angleTolerance) {
			super(driveSubsystem, targetPosition, limeLieghLightSubsystem, angleTolerance);
		}

		public TurnCommand(DriveSubsystem driveSubsystem, String tagID,
				LimeLightSubsystem limeLieghLightSubsystem, double angleTolerance) {
			super(driveSubsystem, tagID, limeLieghLightSubsystem, angleTolerance);
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

	static class DriveDistanceCommand extends frc.robot.commands.drive.DriveDistanceCommand { // more code for debugging
																								// purposes

		protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

		public DriveDistanceCommand(DriveSubsystem driveSubsystem, double targetDistance, double distanceTolerance) {
			super(driveSubsystem, targetDistance, distanceTolerance);
		}

		public DriveDistanceCommand(DriveSubsystem driveSubsystem, Translation2d targetPosition,
				double distanceToTarget,
				LimeLightSubsystem limeLieghLightSubsystem, double distanceTolerance) {
			super(driveSubsystem, targetPosition, distanceToTarget, limeLieghLightSubsystem, distanceTolerance);
		}

		public DriveDistanceCommand(DriveSubsystem driveSubsystem, String tagID, double distanceToTarget,
				LimeLightSubsystem limeLieghLightSubsystem, double distanceTolerance) {
			super(driveSubsystem, tagID, distanceToTarget, limeLieghLightSubsystem, distanceTolerance);
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
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
		if (RobotBase.isSimulation())
			new LimeLightEmulationSubsystem(new Pose(6, 1.45, 0), 0.01, m_driveSubsystem);

		// Configure the button bindings
		m_autoSelector.addOption("Test Steering", SetSteering.getCalibrationCommand(m_driveSubsystem));
		m_autoSelector.addOption("PID Turn 90 degrees", new PIDTurnCommand(m_driveSubsystem, 90, 0.5));
		m_autoSelector.addOption("Bang Bang Drive 2 Meters", new BangBangDriveDistance(m_driveSubsystem, 2, 0.01));
		// m_autoSelector.addOption("PID Drive 2 Meters",
		// DriveDistanceCommand.create(m_driveSubsystem, 3.0, 0.01));
		m_autoSelector.addOption("Knock Over Blocks",
				CommandComposer.getBlocksAuto(m_driveSubsystem));

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
		// new Trigger(() -> DriverStation.getMatchTime() >= 20)
		// .onTrue(m_ArduinoSubsystem.writeStatus(StatusCode.RAINBOW_PARTY_FUN_TIME));
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(
				() -> m_controller.getRawAxis(Axis.kLeftY),
				() -> m_controller.getRawAxis(Axis.kLeftX),
				() -> m_controller.getRawAxis(Axis.kRightTrigger),
				() -> m_controller.getRawAxis(Axis.kLeftTrigger)));
		m_controller.button(Button.kCircle).onTrue(m_driveSubsystem.resetHeadingCommand());
		m_controller.button(Button.kTriangle).onTrue(m_driveSubsystem.alignModulesToZeroComamnd().withTimeout(0.5));
		m_controller.button(Button.kSquare).onTrue(m_driveSubsystem.resetEncodersCommand());
		m_controller.button(Button.kX).onTrue(new DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		Command[] samples = {
				new TurnCommand(m_driveSubsystem, 30, 1)
						.andThen(new TurnCommand(m_driveSubsystem, -30, 1)),
				new TurnCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593),
						m_limeLightSubsystem,
						1),
				new TurnCommand(m_driveSubsystem, "4",
						m_limeLightSubsystem,
						1),
				new DriveDistanceCommand(m_driveSubsystem, 1, 0.05)
						.andThen(new TurnCommand(m_driveSubsystem, -1, 0.05)),
				new DriveDistanceCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593), 1.5,
						m_limeLightSubsystem, 0.05),
				new DriveDistanceCommand(m_driveSubsystem, "4", 1.5,
						m_limeLightSubsystem, 0.05),
				new TurnCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593),
						m_limeLightSubsystem,
						1).andThen(
								new DriveDistanceCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593), 1.5,
										m_limeLightSubsystem, 0.05)),
				new DriveCommandAdvanced(m_driveSubsystem, new Pose(1.0, 0, 0), 0.05, 1)
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(0, 0, 0), 0.05, 1)),
				new DriveCommandAdvanced(m_driveSubsystem, new Pose(0, 0, 0), 0.05, 1)
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(1, 1.0, 0), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(1.44, 1.5, 90), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(1.44, 1.7, 90), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(-0.8, -0.5, -120), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(0, 0, 0), 0.05, 1)),
				new DriveCommandAdvanced(m_driveSubsystem, new Pose(0, 0, 0), 0.05, 1)
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.85, 3.0, 0), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6, 3.0, 0), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.44, 3.5, 90), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.44, 3.7, 90), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(4.2, 1.5, -120), 0.05, 1))
						.andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(0, 0, 0), 0.05, 1))
				// ,
				// new DriveCommandAdvanced(m_driveSubsystem, m_poseEstimationSubsystem, new
				// Translation2d(7.87, 1.45),
				// 1.2, 0.05, 1)
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.0, 0.0, 0),
				// 0.05, 1))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Translation2d(7.87,
				// 1.45), 1.2,
				// 0.05,
				// 1))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.0, 1.45, 0),
				// 0.05, 1))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Translation2d(7.87,
				// 1.45), 1.2,
				// 0.05,
				// 1))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.0, 2.92, 0),
				// 0.05, 1))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Translation2d(7.87,
				// 1.45), 1.2,
				// 0.05,
				// 1)),
				// new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.85, 3.0, 0), 0.05, 1,
				// m_poseEstimationSubsystem)
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6, 3.0, 0),
				// 0.05, 1,
				// m_poseEstimationSubsystem))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.44, 3.5, 90),
				// 0.05, 1,
				// m_poseEstimationSubsystem))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.44, 3.7, 90),
				// 0.05, 1,
				// m_poseEstimationSubsystem))
				// .andThen(new DriveCommandAdvanced(m_driveSubsystem, new Pose(4.2, 1.5, -120),
				// 0.05, 1,
				// m_poseEstimationSubsystem))
				// .andThen(
				// new DriveCommandAdvanced(m_driveSubsystem, new Pose(6.85, 3.0, 0), 0.05, 1,
				// m_poseEstimationSubsystem))
		};
		m_controller.button(Button.kX)
				.whileTrue(samples[6]);
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
