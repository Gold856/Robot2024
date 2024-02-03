// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
	PneumaticHub m_hub = new PneumaticHub();
	DoubleSolenoid m_solenoid = m_hub.makeDoubleSolenoid(1, 0);

	/** Creates a new PneumaticsSubsystem. */
	public PneumaticsSubsystem() {
	}

	public void set() {
		m_solenoid.set(Value.kForward);
		m_hub.setOneShotDuration(0, 100);
	}

	public Command setCommand() {
		return runOnce(this::set);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
