package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Drivetrain {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final double kP = 0.25;
    public static final double kI = 0.001;
    public static final double kD = 20;
    public static final double kF = 1023.0/7200.0;
    public static final double Iz = 300;
    public static final double PeakOut = 1.00;
    public static final double kDeadband = 0.1;
    private final WPI_VictorSPX _leftFront = new WPI_VictorSPX(2);
    private final WPI_VictorSPX _rightFront = new WPI_VictorSPX(1);
    private final WPI_TalonSRX _leftRear = new WPI_TalonSRX(4);
    private final WPI_TalonSRX _rightRear = new WPI_TalonSRX(3);

    public void init(){
        _leftFront.configFactoryDefault();
        _rightFront.configFactoryDefault();
        _leftRear.configFactoryDefault();
        _rightRear.configFactoryDefault();

        _leftFront.follow(_leftRear);
        _rightFront.follow(_rightRear);

        _leftFront.setInverted(InvertType.FollowMaster);
        _rightFront.setInverted(InvertType.FollowMaster);

        _rightRear.setInverted(true);

        _leftRear.setSensorPhase(true);
        _rightRear.setSensorPhase(true);

        configPID(_leftRear);
        configPID(_rightRear);
    }

    public void arcadeDrive(double speed, double rotation){
        var filterspeed = filterInput(speed);
        var filterrotation = filterInput(rotation);

        var leftspeed = Math.max(Math.min(filterspeed + 0.5*filterrotation, 1.0), -1.0);
        var rightspeed = Math.max(Math.min(filterspeed - 0.5*filterrotation, 1.0), -1.0);

        double leftVelocity_UnitsPer100ms = leftspeed *200 *4096.0 /600;
        double rightVelocity_UnitPer100ms = rightspeed *200 *4096.0 /600;

        _leftRear.set(ControlMode.Velocity, leftVelocity_UnitsPer100ms);
        _rightRear.set(ControlMode.Velocity, rightVelocity_UnitPer100ms);
    }

    private void configPID(WPI_TalonSRX talon){
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configPeakOutputForward(1, kTimeoutMs);
        talon.configPeakOutputReverse(-1, kTimeoutMs);

        talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
        talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
        talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
        talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    }

    private double filterInput(double input){
        if(Math.abs(input) <= kDeadband){
            return 0;
        } else {
            if (input < 0) {
                return input * input * -1;
            } else {
                return input * input;
            }
        }
    }
}