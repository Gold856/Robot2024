package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArduinoSubsystem.StatusCode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class CommandComposer {

	public static Command getBlocksAuto(DriveSubsystem m_driveSubsystem) {
		return new SequentialCommandGroup(
				DriveDistanceCommand.create(m_driveSubsystem, 0.75),
				DriveDistanceCommand.create(m_driveSubsystem, -1.25),
				DriveDistanceCommand.create(m_driveSubsystem, 1.5),
				DriveDistanceCommand.create(m_driveSubsystem, -1.825),
				DriveDistanceCommand.create(m_driveSubsystem, 2.125),
				DriveDistanceCommand.create(m_driveSubsystem, -2.5));
	}

	public static Command getForward(PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return parallel(pneumaticsSubsystem.setForwardCommand(), arduinoSubsystem.writeStatus(StatusCode.SOLID_RED));
	}

	public static Command getReverseCommand(PneumaticsSubsystem pneumaticsSubsystem,
			ArduinoSubsystem arduinoSubsystem) {
		return parallel(pneumaticsSubsystem.setReverseCommand(), arduinoSubsystem.writeStatus(StatusCode.SOLID_BLUE));
	}

	public static Command getOffCommand(PneumaticsSubsystem pneumaticsSubsystem, ArduinoSubsystem arduinoSubsystem) {
		return parallel(pneumaticsSubsystem.setOffCommand(), arduinoSubsystem.writeStatus(StatusCode.SOLID_YELLOW));
	}
}
