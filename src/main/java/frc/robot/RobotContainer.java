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
import frc.robot.commands.PIDTurnCommand;
import frc.robot.commands.SetSteering;
import frc.robot.commands.drive.TagAlignCommand;
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

		@Override
		public void periodic() {
			super.periodic();
			var pose = estimatedPose();
			table.getEntry("Pose Estimated").setString("" + pose);
			if (pose != null)
				table.getEntry("BotPose'").setDoubleArray(Pose.toPose2DAdvantageScope(pose));
			table.getEntry("BotPose@Odometry")
					.setDoubleArray(Pose.toPose2DAdvantageScope(m_driveSubsystem.getPose()));
			table.getEntry("BotPose'@Odometry")
					.setDoubleArray(Pose.toPose2DAdvantageScope(m_driveSubsystem.getPose()));
			try {
				pose = new Pose(m_botpose.value[0], m_botpose.value[1], m_botpose.value[5]);
				table.getEntry("BotPose").setDoubleArray(Pose.toPose2DAdvantageScope(pose));
			} catch (Exception e) {
			}
		}

		@Override
		public void recordPose(String entryName, Pose2d value) {
			RobotContainer.this.recordPose(entryName, value);
		}

		@Override
		public void recordString(String entryName, String value) {
			RobotContainer.this.recordString(entryName, value);
		}

	};

	protected NetworkTable table = NetworkTableInstance.getDefault().getTable("AdvantageScope");

	public void recordPose(String entryName, Pose2d value) {
		if (value == null)
			table.getEntry(entryName).setDoubleArray(new double[0]);
		else
			table.getEntry(entryName).setDoubleArray(Pose.toPose2DAdvantageScope(value.getX(),
					value.getY(), value.getRotation().getDegrees()));
	}

	public void recordString(String entryName, String value) {
		table.getEntry(entryName).setString(value);
	}

	class TurnCommand extends frc.robot.commands.drive.TurnCommand { // more code for debugging

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
			RobotContainer.this.recordPose(entryName, value);
		}

		@Override
		public void recordString(String entryName, String value) {
			RobotContainer.this.recordString(entryName, value);
		}

	}

	class DriveDistanceCommand extends frc.robot.commands.drive.DriveDistanceCommand { // more code for debugging

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
			RobotContainer.this.recordPose(entryName, value);
		}

		@Override
		public void recordString(String entryName, String value) {
			RobotContainer.this.recordString(entryName, value);
		}

	}

	class DriveCommand extends frc.robot.commands.drive.DriveCommand { // more code for debugging

		public DriveCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
				double angleTolerance) {
			super(driveSubsystem, targetPose, distanceTolerance, angleTolerance);
		}

		public DriveCommand(DriveSubsystem driveSubsystem, Pose2d targetPose,
				LimeLightSubsystem limeLightSubsystem, double distanceTolerance,
				double angleTolerance) {
			super(driveSubsystem,
					() -> driveSubsystem.getPose().plus(targetPose.minus(limeLightSubsystem.estimatedPose())),
					distanceTolerance, angleTolerance);
		}

		public DriveCommand(DriveSubsystem driveSubsystem, Translation2d targetPosition,
				double distanceToTarget, LimeLightSubsystem limeLightSubsystem, double distanceTolerance,
				double angleTolerance) {

			super(driveSubsystem,
					() -> driveSubsystem.getPose()
							.plus(limeLightSubsystem.getTargetPose(targetPosition, distanceToTarget)
									.minus(limeLightSubsystem.estimatedPose())),
					distanceTolerance, angleTolerance);
		}

		@Override
		public void recordPose(String entryName, Pose2d value) {
			RobotContainer.this.recordPose(entryName, value);
		}

		@Override
		public void recordString(String entryName, String value) {
			RobotContainer.this.recordString(entryName, value);
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
		// m_controller.button(Button.kX).onTrue(new
		// DriveDistanceCommand(m_driveSubsystem, 10, 0.01));
		Command[] samples = {
				new DriveCommand(m_driveSubsystem, new Pose(1.0, 0, 0), 0.2, 5)
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(1, 1, 0), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(1, 1, 45), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(0, 0, 0), 0.2, 5)),
				new TurnCommand(m_driveSubsystem, 45, 5)
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5)),
				new DriveDistanceCommand(m_driveSubsystem, 1, 0.2)
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.2)),
				new TurnCommand(m_driveSubsystem, 45, 5)
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, 1, 0.2))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.2))
						.andThen(new TurnCommand(m_driveSubsystem, 45, 5))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, 1, 0.2))
						.andThen(new DriveDistanceCommand(m_driveSubsystem, -1, 0.2))
						.andThen(new TurnCommand(m_driveSubsystem, -45, 5)),
				new DriveCommand(m_driveSubsystem, new Translation2d(7.87, 1.45),
						1.2, m_limeLightSubsystem, 0.2, 5)
								.andThen(new DriveCommand(m_driveSubsystem, new Pose(6.0, 0.0, 0), m_limeLightSubsystem,
										0.2, 5))
								.andThen(new DriveCommand(m_driveSubsystem, new Translation2d(7.87,
										1.45), 1.2, m_limeLightSubsystem,
										0.05,
										1))
								.andThen(
										new DriveCommand(m_driveSubsystem, new Pose(6.0, 1.45, 0), m_limeLightSubsystem,
												0.2, 5))
								.andThen(new DriveCommand(m_driveSubsystem, new Translation2d(7.87,
										1.45), 1.2, m_limeLightSubsystem,
										0.05,
										1))
								.andThen(
										new DriveCommand(m_driveSubsystem, new Pose(6.0, 2.92, 0), m_limeLightSubsystem,
												0.2, 5))
								.andThen(new DriveCommand(m_driveSubsystem, new Translation2d(7.87,
										1.45), 1.2, m_limeLightSubsystem,
										0.05,
										1)),
				new TurnCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593),
						m_limeLightSubsystem, 5).andThen(
								new DriveDistanceCommand(m_driveSubsystem, new Translation2d(8.308467, 1.442593), 1.2,
										m_limeLightSubsystem, 0.2)),
				new TurnCommand(m_driveSubsystem, "4",
						m_limeLightSubsystem,
						3).andThen(
								new DriveDistanceCommand(m_driveSubsystem, "4", 1.5,
										m_limeLightSubsystem, 0.2)),
				new DriveCommand(m_driveSubsystem, new Pose(1.0, 0, 0), 0.2, 5)
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(1, 1, 0), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(1, 1, 45), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(0, 0, 0), 0.2, 5)),
				new DriveCommand(m_driveSubsystem, new Pose(0, 0, 0), 0.2, 5)
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(6.85, 3.0, 0), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(6, 3.0, 0), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(6.44, 3.5, 90), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(6.44, 3.7, 90), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(4.2, 1.5, -120), 0.2, 5))
						.andThen(new DriveCommand(m_driveSubsystem, new Pose(0, 0, 0), 0.2, 5)),
				new TagAlignCommand(m_driveSubsystem, m_limeLightSubsystem, 5)
		};
		m_controller.button(Button.kX)
				.whileTrue(samples[0]);
	}

	public Command getAutonomousCommand() {
		return m_autoSelector.getSelected();
	}
}
