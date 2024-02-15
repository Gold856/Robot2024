// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
	private final PneumaticHub m_hub = new PneumaticHub(kPneumaticHubID);
	private final DoubleSolenoid m_ampBarSolenoid = m_hub.makeDoubleSolenoid(kAmpBarForwardChannel,
			kAmpBarReverseChannel);
	private final DoubleSolenoid m_intakeSolenoid = m_hub.makeDoubleSolenoid(kIntakeForwardChannel,
			kIntakeReverseChannel);
	private boolean m_ampBarExtended;
	private boolean m_intakeExtended;

	/** Creates a new PneumaticsSubsystem. */
	public PneumaticsSubsystem() {
	}

	/**
	 * Creates a command to toggle the intake.
	 * 
	 * @return The command.
	 */
	public Command toggleIntakeCommand() {
		return runOnce(() -> {
			if (!m_intakeExtended) {
				m_intakeSolenoid.set(kIntakeUp);
				m_intakeExtended = true;
			} else {
				m_intakeSolenoid.toggle();
			}
		});
	}

	/**
	 * Creates a command to raise the intake.
	 * 
	 * @return The command.
	 */
	public Command upIntakeCommand() {
		// TODO direction/starting state?
		return runOnce(() -> m_intakeSolenoid.set(kIntakeUp));
	}

	/**
	 * Creates a command to raise the intake.
	 * 
	 * @return The command.
	 */
	public Command downIntakeCommand() {
		// TODO direction/starting state?
		return runOnce(() -> m_intakeSolenoid.set(kIntakeDown));
	}

	/**
	 * Creates a command to toggle the amp bar.
	 * 
	 * @return The command.
	 */
	public Command toggleAmpBarCommand() {
		return runOnce(() -> {
			if (!m_ampBarExtended) {
				m_ampBarSolenoid.set(Value.kForward);
				m_ampBarExtended = true;
			} else {
				m_ampBarSolenoid.toggle();
			}
		});
	}

	/**
	 * Creates a command to extend the amp bar.
	 * 
	 * @return The command.
	 */
	public Command extendAmpBarCommand() {
		return runOnce(() -> m_ampBarSolenoid.set(Value.kForward));
	}

	/**
	 * Creates a command to retract the amp bar.
	 * 
	 * @return The command.
	 */
	public Command retractAmpBarCommand() {
		return runOnce(() -> m_ampBarSolenoid.set(Value.kReverse));
	}
}
