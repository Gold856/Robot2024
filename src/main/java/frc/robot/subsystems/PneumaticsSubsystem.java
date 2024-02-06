// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
	private final PneumaticHub m_hub = new PneumaticHub(50);
	private final DoubleSolenoid m_ampBarSolenoid = m_hub.makeDoubleSolenoid(3, 4);
	private final DoubleSolenoid m_intakeSolenoid = m_hub.makeDoubleSolenoid(1, 2);

	/** Creates a new PneumaticsSubsystem. */
	public PneumaticsSubsystem() {
		m_hub.setOneShotDuration(0, 100);
	}

	/**
	 * Creates a command to toggle the intake.
	 * 
	 * @return A command to toggle the intake.
	 */
	public Command toggleIntakeCommand() {
		return runOnce(m_intakeSolenoid::toggle);
	}

	/**
	 * Creates a command to toggle the amp bar.
	 * 
	 * @return A command to toggle the amp bar.
	 */
	public Command toggleAmpBarCommand() {
		return runOnce(m_ampBarSolenoid::toggle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
