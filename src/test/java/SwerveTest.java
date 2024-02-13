import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTest {
	static DriveSubsystem m_driveSubsystem;

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
		m_driveSubsystem = new DriveSubsystem();
	}

	@Test
	void testModuleStates() {
		var expectedStates = new SwerveModuleState[4];

		// Go Northwest
		var nwStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(1, 1,
				0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1,
				Rotation2d.fromDegrees(45)));
		assertArrayEquals(expectedStates, nwStates);

		// Go Northeast
		var neStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(1,
				-1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1,
				Rotation2d.fromDegrees(-45)));
		assertArrayEquals(expectedStates, neStates);

		// Go Southwest
		var swStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(-1,
				1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1,
				Rotation2d.fromDegrees(135)));
		assertArrayEquals(expectedStates, swStates);

		// Go Southwest
		var seStates = m_driveSubsystem.calculateModuleStates(new ChassisSpeeds(-1,
				-1, 0), false);
		Arrays.fill(expectedStates, new SwerveModuleState(1,
				Rotation2d.fromDegrees(225)));
		assertArrayEquals(expectedStates, seStates);
	}

	@Test
	void testDriveTimeCommand() {
		// DriverStationSim.setEnabled(true);
		// DriverStation.refreshData();
		// var command = new DriveTimeCommand(m_driveSubsystem, 1, 1);
		// command.schedule();
		// SimHooks.pauseTiming();
		// for (int i = 0; i < 60; i++) {
		// CommandScheduler.getInstance().run();
		// SimHooks.stepTiming(0.02);
		// }
		// assertEquals(3.0, m_driveSubsystem.getPose().getX(), .1);
	}

	@Test
	void testPose() {
		System.out
				.println(m_driveSubsystem.getPose() + "     "
						+ Arrays.toString(m_driveSubsystem.getModulePositions()));
		for (int i = 0; i < 5; i++) {
			m_driveSubsystem.setModuleStates(0.5, 0, 0, true);
		}
		System.out
				.println(m_driveSubsystem.getPose() + "     "
						+ Arrays.toString(m_driveSubsystem.getModulePositions()));
		for (int i = 0; i < 11; i++) {
			m_driveSubsystem.setModuleStates(0, 0, 0.3, true);
		}
		System.out
				.println(m_driveSubsystem.getPose() + "     "
						+ Arrays.toString(m_driveSubsystem.getModulePositions()));
	}

}
