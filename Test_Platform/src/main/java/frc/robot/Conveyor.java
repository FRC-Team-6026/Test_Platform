package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor {
    private final AnalogInput _ultraSonic = new AnalogInput(0);

    public void init(){
        _ultraSonic.initAccumulator();
    }

    public boolean isBallInLoadingPosition(){
        SmartDashboard.putNumber("Ultrasonic Output", _ultraSonic.getVoltage());
        return false;
    }

    public void runForward(){

    }

    public void runBackward(){
        
    }
}