package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake {

    private final WPI_VictorSPX _intake = new WPI_VictorSPX(14);

    public void init(){
        _intake.configFactoryDefault();
    }

    public boolean isExtended(){
        return false;
    }

    /**
     * Extends the intake mechanism outside the robot
     */
    public void extend(){
        
    }

    /**
     * Retracts the intake mechanism inside the robot
     */
    public void retract(){

    }

    /**
     * Run motors of the intake
     */
    public void run(){
        _intake.set(ControlMode.PercentOutput, 0.4);
    }

    /**
     * Stop the intake motor
     */
    public void stop(){
        _intake.stopMotor();
    }

    /**
     * Run the intake backwards
     */
    public void reverse(){
        _intake.set(ControlMode.PercentOutput, -0.4);
    }
}