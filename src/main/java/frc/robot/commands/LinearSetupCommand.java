// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterAndFlywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LinearRangeFinder;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LinearSetupCommand extends Command {

	private double m_flywheelSetpoint, m_actuatorHeightSetpoint;
	private ShooterSubsystem m_shooterSubsystem;
	private FlywheelSubsystem m_flywheelSubsystem;
	private String m_name;
	private LinearRangeFinder m_distanceClass;
	private boolean m_preset;
	private double m_distance, m_angle;

	/**
	 * Setup the flywheel and hood
	 * 
	 * @param shooterSubsystem
	 * @param flywheelSubsystem
	 * @param m_distance        The distance between the robot and the speaker
	 * @param shootClass        The type of shooting, currently only takes in LINEAR
	 */
	public LinearSetupCommand(ShooterSubsystem shooterSubsystem, FlywheelSubsystem flywheelSubsystem, double distance,
			String shootClass) {
		m_distance = distance;
		m_name = shootClass;
		m_shooterSubsystem = shooterSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;
		m_preset = false; // does not take in fixed velocity and angle
		addRequirements(m_shooterSubsystem, m_flywheelSubsystem);
	}

	/**
	 * Set the setpoints of the flywheel and hood
	 */
	public void initialize() {
		// if (m_preset) {
		// m_flywheelSubsystem.setVelocity(m_flywheelSetpoint);
		// m_hoodSubsystem.setPosition(m_hoodSetpoint);
		// } else {
		if (m_name.equals("LINEAR")) {
			m_distanceClass = new LinearRangeFinder();
			// } else if (m_name.equals("REGRESSION")) {
			// m_distanceClass = new RegressionRangeFinder();
			// } IF WE NEED QUAD REGRESSION

			m_angle = m_shooterSubsystem.calculateAngle(m_distance);
			m_flywheelSetpoint = m_distanceClass.getAngleAndRPM(m_distance)[1];

			// m_flywheelSubsystem.setVelocity(m_flywheelSetpoint);
			// m_actuatorHeightSetpoint =
			// m_shooterSubsystem.calcActuatorHeightFromAngle(m_angle);
			m_flywheelSubsystem.setSpeed(m_flywheelSetpoint);
			m_shooterSubsystem.setActuatorHeight(m_actuatorHeightSetpoint);
		}
	}

	/**
	 * Stop the flywheel and reset the hood at the end of the command
	 */
	public void end(boolean interrupted) {
		m_flywheelSubsystem.setSpeed(0);
		m_shooterSubsystem.setActuatorHeight(0); // TODO set actuator height
	}
}