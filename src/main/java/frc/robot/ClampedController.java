package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/**
 * A Clamped controller in extension of PIDs
 * 
 * @author Nitya Bajaj
 */
public class ClampedController extends PIDController {
	private final double m_minPower;
	private final double m_maxPower;

	/**
	 * Constructor for the clamped controller
	 * 
	 * @param p        The P value
	 * @param minPower The min power
	 * @param maxPower The max power
	 */
	public ClampedController(double p, double minPower, double maxPower) {
		super(p, 0.0, 0.0);
		m_minPower = minPower;
		m_maxPower = maxPower;
	}

	/**
	 * Abs of the power, if greater than max power, max power * the signum of power,
	 * if less than min power than min power * the signum of power
	 * 
	 * @param power
	 * @return The power
	 */
	double clamp(double power) {
		if (Math.abs(power) > m_maxPower) {
			return m_maxPower * Math.signum(power);
		}
		if (Math.abs(power) < m_minPower) {
			return m_minPower * Math.signum(power);
		}
		return power;
	}

	/**
	 * Uses Clamp to take the clamp of the next output of the PID controller (from
	 * calculate)
	 * 
	 * @param currentValue The current value
	 * @param setpoint     The setpoint
	 * @return The power
	 */
	@Override
	public double calculate(double currentValue, double setpoint) {
		return clamp(super.calculate(currentValue, setpoint));
	}

	/**
	 * Uses clamp to take the clamp of the next output of the PID controller without
	 * the setpoint
	 * 
	 * @param currentValue
	 * @return The power
	 */
	@Override
	public double calculate(double currentValue) {
		return clamp(super.calculate(currentValue));
	}
}
