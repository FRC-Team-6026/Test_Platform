package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private final WPI_TalonSRX _top = new WPI_TalonSRX(99);
    private final WPI_TalonSRX _bottom = new WPI_TalonSRX(98);
    private final int _pidSlot = 0;
    private final int _timeoutMs = 30;
    private final double _p = 0.2;
    private final double _i = 0;
    private final double _d = 0;
    private final double _f = 0.2;
    private final double _peakOutput = 1;

    public void init(){
        _top.configFactoryDefault();
        _bottom.configFactoryDefault();

        _top.setNeutralMode(NeutralMode.Brake);
        _bottom.setNeutralMode(NeutralMode.Brake);

        _top.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, _pidSlot, _timeoutMs);
        _bottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, _pidSlot, _timeoutMs);

        _top.setInverted(false);
        _top.setSensorPhase(false);
        _bottom.setInverted(false);
        _bottom.setSensorPhase(false);

        _top.configNominalOutputForward(0, _timeoutMs);
        _top.configNominalOutputReverse(0, _timeoutMs);
        _top.configPeakOutputForward(_peakOutput, _timeoutMs);
        _top.configPeakOutputReverse(-_peakOutput, _timeoutMs);

        _bottom.configNominalOutputForward(0, _timeoutMs);
        _bottom.configNominalOutputReverse(0, _timeoutMs);
        _bottom.configPeakOutputForward(_peakOutput, _timeoutMs);
        _bottom.configPeakOutputReverse(-_peakOutput, _timeoutMs);

        _top.config_kF(_pidSlot, _f, _timeoutMs);
        _top.config_kP(_pidSlot, _p, _timeoutMs);
        _top.config_kI(_pidSlot, _i, _timeoutMs);
        _top.config_kD(_pidSlot, _d, _timeoutMs);

        _bottom.config_kF(_pidSlot, _f, _timeoutMs);
        _bottom.config_kP(_pidSlot, _p, _timeoutMs);
        _bottom.config_kI(_pidSlot, _i, _timeoutMs);
        _bottom.config_kD(_pidSlot, _d, _timeoutMs);

    }

    /**
     * Runs the firing mechanism at the set power
     * @param power power output 0 to 1
     */
    public void fire(double power){
        _top.set(ControlMode.PercentOutput, power);
        _bottom.set(ControlMode.PercentOutput, power);

        SmartDashboard.putNumber("Top velocity pulses/100ms", _top.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Bottom velocity pulses/100ms", _bottom.getSelectedSensorVelocity());
    }

    public boolean isAtSetPower(double power){
        return false;
    }

    /**
     * Opens the gate
     */
    public void openGate(){
        
    }

    /**
     * Closes the gate
     */
    public void closeGate(){

    }
}