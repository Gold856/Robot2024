import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTest {
	static DriveSubsystem m_driveSubsystem;

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
		m_driveSubsystem = new DriveSubsystem();
	}

	@AfterAll
	static void shutdown() {
		m_driveSubsystem.close();
	}

	@Test
	void testModuleStates() {
		var expectedStates = new SwerveModuleState[4];

		// Go Northwest
		var nwStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(1, 1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(45)));
		assertArrayEquals(expectedStates, nwStates);

		// Go Northeast
		var neStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(1, -1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(-45)));
		assertArrayEquals(expectedStates, neStates);

		// Go Southwest
		var swStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(-1, 1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(135)));
		assertArrayEquals(expectedStates, swStates);

		// Go Southwest
		var seStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(-1, -1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1, Rotation2d.fromDegrees(225)));
		assertArrayEquals(expectedStates, seStates);
	}

	@Test
	void testDriveTimeCommand() {
		DriverStationSim.setEnabled(true);
		DriverStation.refreshData();
		var command = new DriveTimeCommand(m_driveSubsystem, 1, 1);
		command.schedule();
		SimHooks.pauseTiming();
		for (int i = 0; i < 52; i++) {
			CommandScheduler.getInstance().run();
			SimHooks.stepTiming(0.02);
		}
		System.out.println();
		assertEquals(1.02, m_driveSubsystem.getPose().getX(), 1e-9);
		SimHooks.resumeTiming();
	}
}
