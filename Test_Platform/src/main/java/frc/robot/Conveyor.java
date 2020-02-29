package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conveyor {
    private final I2C.Port _i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 _colorSensor = new ColorSensorV3(_i2cPort);
    private final VictorSPX _motor = new VictorSPX(99);

    public void init(){
        _motor.configFactoryDefault();
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

    public void run(double percentOutput){
        _motor.set(ControlMode.PercentOutput, percentOutput);
    }
}