// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int LElevatorMotor = 13;
    public static final int RElevatorMotor = 14;

    public static final int ARM_MOTOR = 15;
    public static final int WRIST_MOTOR = 16;
	public static final int INTAKE_MOTOR = 17;

	public static final int EncoderChannelA = 0;	//TODO
	public static final int EncoderChannelB = 0;	//TODO

    public static final double DEADZONE_VALUE = 0.01;
	public static final int NUMBER_OF_CONTROLLERS = 2;

	public static final double Elevator_kp = 0;	//TODO
	public static final double Elevator_ki = 0;	//TODO
	public static final double Elevator_kd = 0;	//TODO

	public static final double Arm_kp = 0;	//TODO
    public static final double Arm_ki = 0;	//TODO
    public static final double Arm_kd = 0;	//TODO

	public static final double Wrist_kp = 0;	//TODO
    public static final double Wrist_ki = 0;	//TODO
    public static final double Wrist_kd = 0;	//TODO

    public static final double rElevatorSpeed = 1.0;
	public static final DigitalSource ArmEncoderChannelA = null;
    public static final DigitalSource ArmEncoderChannelB = null;
    public static final DigitalSource WristEncoderChannelA = null;
    public static final DigitalSource WristEncoderChannelB = null;

    public enum Axes {
		LEFT_STICK_X(0), LEFT_STICK_Y(4), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_STICK_X(1), RIGHT_STICK_Y(5);

		private final int value;

		Axes(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum Buttons {
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
				7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

		private final int value;

		private Buttons(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

	}   
}