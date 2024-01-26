import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.aster.commands.drive.TurnCommand;
import frc.aster.subsystems.DriveSubsystem;
import frc.common.PoseEstimationSubsystem;

@TestMethodOrder(OrderAnnotation.class)
public class WestCoastTest {

	static DriveSubsystem m_driveSubsystem;

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
		m_driveSubsystem = new DriveSubsystem();
		// DriverStationSim.setEnabled(true);
		// DriverStation.refreshData();
	}

	@AfterAll
	static void shutdown() {
		// DriverStationSim.setEnabled(false);
		m_driveSubsystem.close();
		// HAL.shutdown();
	}

	@Test
	void testOdometry() {
		var pose = PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose());
		System.out.println(pose);
	}

	@Test
	void testTurnCommand() {
		// System.out.println(PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose()));
		// var command = new TurnCommand(10, 1);
		// command.schedule();
		// SimHooks.pauseTiming();
		// for (int i = 0; i < 52; i++) {
		// CommandScheduler.getInstance().run();
		// SimHooks.stepTiming(0.02);
		// System.out.println(i + ": " +
		// PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose()));
		// }
		// SimHooks.resumeTiming();
	}

	@Test
	void testTurnCommand2() {
		// System.out.println(PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose()));
		// var command = new TurnCommand(10, 1);
		// command.schedule();
		// SimHooks.pauseTiming();
		// for (int i = 0; i < 52; i++) {
		// CommandScheduler.getInstance().run();
		// SimHooks.stepTiming(0.02);
		// System.out.println(i + ": " +
		// PoseEstimationSubsystem.toPose(m_driveSubsystem.getPose()));
		// }
		// SimHooks.resumeTiming();
	}

}
