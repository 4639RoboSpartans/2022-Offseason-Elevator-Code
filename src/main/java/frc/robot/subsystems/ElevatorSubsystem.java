package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    // private TalonSRX ;
    private WPI_VictorSPX lElevatorMotor = new WPI_VictorSPX(Constants.LElevatorMotor);
    private WPI_VictorSPX rElevatorMotor = new WPI_VictorSPX(Constants.RElevatorMotor);

    private Encoder encoder = new Encoder(Constants.EncoderChannelA, Constants.EncoderChannelB);

    public PIDController pid = new PIDController(Constants.Elevator_kp, Constants.Elevator_ki, Constants.Elevator_kd);
    
    public ElevatorSubsystem() {
        lElevatorMotor.configFactoryDefault();
        rElevatorMotor.configFactoryDefault();

        encoder.setDistancePerPulse(1./256);
        encoder.reset();
        pid.setSetpoint(encoder.getDistance());
    }

    public void setPosition(double d){
        double setPoint = d;
        pid.setSetpoint(setPoint);
    }

    @Override
    public void periodic() {
        setSpeed(pid.calculate(encoder.getDistance()));
        SmartDashboard.putNumber("Raw Encoder Distance", encoder.getDistance());
        SmartDashboard.putNumber("Raw Encoder Rate", encoder.getRate());
    }

    public void stop(){
        lElevatorMotor.set(0.0);
        rElevatorMotor.set(0.0);
    }
    
    private void setSpeed(double speed){
        lElevatorMotor.set(speed);
        rElevatorMotor.set(-speed * Constants.rElevatorSpeed);
    }

    public double getSetPoint(){
        return pid.getSetpoint();
    }
}
