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
	PneumaticHub m_hub = new PneumaticHub(50);
	DoubleSolenoid m_solenoid = m_hub.makeDoubleSolenoid(0, 1);

	/** Creates a new PneumaticsSubsystem. */
	public PneumaticsSubsystem() {
		m_hub.setOneShotDuration(0, 100);
	}

	public void setForward() {
		m_solenoid.set(Value.kForward);
	}

	public void setReverse() {
		m_solenoid.set(Value.kReverse);
	}

	public void setOff() {
		m_solenoid.set(Value.kOff);
	}

	public Command setForwardCommand() {
		return runOnce(this::setForward);
	}

	public Command setReverseCommand() {
		return runOnce(this::setReverse);
	}

	public Command setOffCommand() {
		return runOnce(this::setOff);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
