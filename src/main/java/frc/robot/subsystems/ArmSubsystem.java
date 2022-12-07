package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private final WPI_VictorSPX armMotor, wristMotor;

    private final PIDController armPID, wristPID;

    private final Encoder armEncoder, wristEncoder;

    public ArmSubsystem(){
        armMotor = new WPI_VictorSPX(Constants.ARM_MOTOR);
        wristMotor = new WPI_VictorSPX(Constants.WRIST_MOTOR);

        armMotor.configFactoryDefault();
        wristMotor.configFactoryDefault();

        armMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setNeutralMode(NeutralMode.Brake);

        armPID = new PIDController(Constants.Arm_kp, Constants.Arm_ki, Constants.Arm_kd);
        wristPID = new PIDController(Constants.Wrist_kp, Constants.Wrist_ki, Constants.Wrist_kd);

        armEncoder = new Encoder(Constants.ArmEncoderChannelA, Constants.ArmEncoderChannelB);
        wristEncoder = new Encoder(Constants.WristEncoderChannelA, Constants.WristEncoderChannelB);

        armPID.setSetpoint(armEncoder.get());
        wristPID.setSetpoint(wristEncoder.get());
    }

    @Override
    public void periodic() {
        armMotor.set(armPID.calculate(getArmRotation()));
        wristMotor.set(armPID.calculate(getWristRotation()));
    }

    public double getArmRotation(){
        return armEncoder.get() * 1.0;
    }

    public double getWristRotation(){
        return wristEncoder.get() * 1.0;
    }

    public void set(double armPosition, double wristPosition){
        armPID.setSetpoint(armPosition);
        wristPID.setSetpoint(wristPosition);
    }

    public void rotate(double armRotation, double wristRotation){
        armPID.setSetpoint(getArmRotation() + armRotation);
        wristPID.setSetpoint(getWristRotation() + wristRotation);
    }
}