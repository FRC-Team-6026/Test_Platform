package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor {
    private final I2C.Port _i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 _colorSensor = new ColorSensorV3(_i2cPort);

    public void init(){
    }

    public boolean isBallInLoadingPosition(){
        var r = _colorSensor.getRed();
        var g = _colorSensor.getGreen();
        var b = _colorSensor.getBlue();
        var x = _colorSensor.getProximity();
        SmartDashboard.putNumber("red", r);
        SmartDashboard.putNumber("green", g);
        SmartDashboard.putNumber("blue", b);
        SmartDashboard.putNumber("proximity", x);
        if (x > 115 && r > 150 && g > 150){
            return true;
        }
        return false;
    }

    public void runForward(){

    }

    public void runBackward(){
        
    }
}