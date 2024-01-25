package frc.common;

import java.io.File;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * An instance of {@link RobotContainer} is responsible for creating subsystems,
 * commands, and button mappings tailored to a particular robot. This
 * {@link RobotContainer} interface has multiple implementations (e.g.,
 * {@link frc.robot.RobotContainer} and {@link frc.aster.RobotContainer}
 * classes), each corresponding to a different robot type.
 * Through this approach, the {@link frc.robot.Robot} class can easily support
 * any of these robots by instantiating the appropriate implementaton of
 * {@link RobotContainer}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public interface RobotContainer {

	/**
	 * The path to the "deploy" directory in the project.
	 */
	public final static String s_deployPath = "." + File.separator + "src" + File.separator + "main" + File.separator
			+ "deploy";

	/**
	 * Returns the {@code Command} to execute during the autonomous mode.
	 * 
	 * @return the {@code Command} to execute during the autonomous mode
	 */
	Command getAutonomousCommand();
}
