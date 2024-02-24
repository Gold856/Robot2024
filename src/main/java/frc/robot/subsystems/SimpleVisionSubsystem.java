package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleVisionSubsystem extends SubsystemBase {
	private final DoubleSubscriber m_subscription;
	private double m_angle;

	/*
	 * Controller used for turn to tag in teleop
	 */
	private final PIDController m_controller;

	public SimpleVisionSubsystem() {
		m_controller = new PIDController(0.1, 0, 0);
		m_controller.enableContinuousInput(0, 360);

		// Setup subscription
		m_subscription = NetworkTableInstance.getDefault().getTable("limelight")
				.getDoubleTopic("tx")
				.subscribe(0);
	}

	@Override
	public void periodic() {
		// get the data from limelight
		m_angle = m_subscription.get();
		SmartDashboard.putNumber("limelight angle to turn", m_angle);
	}

	/**
	 * Returns the angle of the camera to the tag. Negative if the tag is left of
	 * the camera, positive if the tag is right of the camera.
	 * 
	 * @return The angle of the camera
	 */
	public double getAngle() {
		return m_angle;
	}

	public Supplier<Double> getTurnSupplier() {

		return () -> {
			// Get angle to tag from VisionSubsystem
			double currentAngle = getAngle();
			// Set power using PIDController
			double power = m_controller.calculate(currentAngle);
			double powerLimit = 0.75;
			power = MathUtil.clamp(power, -powerLimit, powerLimit);

			SmartDashboard.putNumber("tag power?", -power);
			return -power;
		};
	}
}
