package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.DriveSubsystem;

public class CommandComposer {
	public static Command knockDownBlocks(DriveSubsystem subsystem) {
		double tolerance = 0.01;
		return sequence(new DriveDistanceCommand(subsystem, 0.83, tolerance),
				new DriveDistanceCommand(subsystem, -1.72, tolerance),
				new DriveDistanceCommand(subsystem, 1.95, tolerance),
				new DriveDistanceCommand(subsystem, -2.02, tolerance),
				new DriveDistanceCommand(subsystem, 2.2, tolerance),
				new DriveDistanceCommand(subsystem, -2.37, tolerance));
	}
}
