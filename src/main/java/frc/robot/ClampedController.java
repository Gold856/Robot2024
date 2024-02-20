package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class ClampedController extends PIDController {
	private final double m_minPower;
	private final double m_maxPower;

	public ClampedController(double p, double minPower, double maxPower) {
		super(p, 0.0, 0.0);
		m_minPower = minPower;
		m_maxPower = maxPower;
	}

	double clamp(double power) {
		if (Math.abs(power) > m_maxPower) {
			return m_maxPower * Math.signum(power);
		}
		if (Math.abs(power) < m_minPower) {
			return m_minPower * Math.signum(power);
		}
		return power;
	}

	@Override
	public double calculate(double currentValue, double setpoint) {
		return clamp(super.calculate(currentValue, setpoint));
	}

	@Override
	public double calculate(double currentValue) {
		return clamp(super.calculate(currentValue));
	}
}