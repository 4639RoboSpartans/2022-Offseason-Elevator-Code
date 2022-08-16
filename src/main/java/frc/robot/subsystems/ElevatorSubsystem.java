package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    // private TalonSRX ;
    private WPI_VictorSPX lElevatorMotor = new WPI_VictorSPX(Constants.LElevatorMotor);
    private WPI_VictorSPX rElevatorMotor = new WPI_VictorSPX(Constants.RElevatorMotor);

    public ElevatorSubsystem() {
        lElevatorMotor.configFactoryDefault();
        rElevatorMotor.configFactoryDefault();
    }

    public void set(double speed){
        lElevatorMotor.set(speed);
        rElevatorMotor.set(-speed * Constants.rElevatorSpeed);
    }

    public void stop(){
        lElevatorMotor.set(0);
        rElevatorMotor.set(0);
    }
}
