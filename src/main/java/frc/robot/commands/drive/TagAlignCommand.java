package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class TagAlignCommand extends TurnCommand {
	public TagAlignCommand(DriveSubsystem driveSubsystem,
			LimeLightSubsystem limeLightSubsystem, double angleTolerance) {
		super(driveSubsystem, limeLightSubsystem.getRotationToDetectedTags().getDegrees(), angleTolerance);
	}
}
