// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
	/**
	 * The I2C device we're connecting to. Port.kMXP means we use the I2C connection
	 * on the MXP port, which runs through the navX
	 */
	private final SerialPort usb = new SerialPort(9600, Port.kUSB);

	/** The bytes that control the LED mode */
	public enum StatusCode {
		RESET((byte) 8),
		BLINKING_NOTE((byte) 9),
		RAINBOW_PARTY_FUN_TIME((byte) 10),
		DEFAULT((byte) 20);

		public byte code;

		private StatusCode(byte c) {
			code = c;
		}
	}

	/** Creates a new ArduinoSubsystem. */
	public ArduinoSubsystem() {
		setCode(StatusCode.DEFAULT);
	}

	public void setCode(StatusCode code) {
		usb.write(new byte[] { code.code }, 1);
	}

	public Command writeStatus(StatusCode code) {
		return runOnce(() -> setCode(code));
	}
}