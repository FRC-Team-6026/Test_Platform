package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
    private static final double kP = 5e-5;
    private static final double kI = 1e-6;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double kDeadband = 0.1;
    private static final double kMaxRPM = 1000;
    private final CANSparkMax _leftFront = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _rightFront = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax _leftRear = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax _rightRear = new CANSparkMax(8, MotorType.kBrushless);
    private final CANPIDController _leftController = _leftRear.getPIDController();
    private final CANPIDController _rightController = _rightRear.getPIDController();
    private final CANEncoder _leftCanEncoder = _leftRear.getEncoder();
    private final CANEncoder _rightCanEncoder = _rightRear.getEncoder();

    public void init(){
        _leftFront.restoreFactoryDefaults();
        _rightFront.restoreFactoryDefaults();
        _leftRear.restoreFactoryDefaults();
        _rightRear.restoreFactoryDefaults();

        _leftFront.follow(_leftRear);
        _rightFront.follow(_rightRear);

        _rightRear.setInverted(true);

        //_leftCanEncoder.setInverted(true);
        //_rightCanEncoder.setInverted(true);

        config(_leftController, _leftCanEncoder);
        config(_rightController, _rightCanEncoder);
    }

    public void arcadeDrive(double speed, double rotation){
        var filterspeed = filterInput(speed);
        var filterrotation = filterInput(rotation);

        var leftspeed = Math.max(Math.min(filterspeed + 0.5*filterrotation, 1.0), -1.0);
        var rightspeed = Math.max(Math.min(filterspeed - 0.5*filterrotation, 1.0), -1.0);

        double leftVelocity_RPM = leftspeed * kMaxRPM;
        double rightVelocity_RPM = rightspeed * kMaxRPM;

        SmartDashboard.putNumber("speed", filterspeed);
        SmartDashboard.putNumber("rotation", filterrotation);

        SmartDashboard.putNumber("Left target RPM", leftVelocity_RPM);
        SmartDashboard.putNumber("right target RPM", rightVelocity_RPM);

        SmartDashboard.putNumber("left encoder velocity", _leftCanEncoder.getVelocity());
        SmartDashboard.putNumber("right encoder velocity", _rightCanEncoder.getVelocity());

        SmartDashboard.putNumber("right front current", _rightFront.getOutputCurrent());
        SmartDashboard.putNumber("right rear current", _rightRear.getOutputCurrent());
        SmartDashboard.putNumber("left front current", _leftFront.getOutputCurrent());
        SmartDashboard.putNumber("left rear current", _leftRear.getOutputCurrent());

        SmartDashboard.putNumber("right front temp", _rightFront.getMotorTemperature());
        SmartDashboard.putNumber("right rear temp", _rightRear.getMotorTemperature());
        SmartDashboard.putNumber("left front temp", _leftFront.getMotorTemperature());
        SmartDashboard.putNumber("left rear temp", _leftRear.getMotorTemperature());

        _leftController.setReference(leftVelocity_RPM, ControlType.kVelocity);
        _rightController.setReference(rightVelocity_RPM, ControlType.kVelocity);
    }

    private void config(CANPIDController controller, CANEncoder encoder){
        //encoder.setVelocityConversionFactor(1.0/4096.0);
        controller.setFF(kF);
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setOutputRange(-.5, .5);
        controller.setFeedbackDevice(encoder);
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