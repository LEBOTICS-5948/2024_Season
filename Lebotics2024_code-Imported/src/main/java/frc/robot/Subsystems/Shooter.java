package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Intake.IntakeState;

public class Shooter extends SubsystemBase{
    private static Shooter m_instance = null;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter();
        }
        return m_instance;
    }

    public enum ShooterState{
        START,
        SHOOT,
        LOAD,
        AIMBOT,
        AIM,
        STOP,
        HOLD,
    }

    private final CANSparkMax r_ShooterMotor = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax l_ShooterMotor = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax FeederMotor = new CANSparkMax(18, MotorType.kBrushless);

    private final CANSparkMax angleMotor1 = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax angleMotor2 = new CANSparkMax(16, MotorType.kBrushless);
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePIDController;

    private final RelativeEncoder r_ShooterEncoder;
    private final RelativeEncoder l_ShooterEncoder;

    private final SparkPIDController r_PIDController;
    private final SparkPIDController l_PIDController;

    private final Intake intake = Intake.getInstance();

    private ShooterState state, lastState;

    public boolean isReady, isLoaded = false;

    private Rev2mDistanceSensor distMXP;

    //private double targetRPM; //shooter_speed

    @Override
    public void periodic(){
        if((state.equals(ShooterState.LOAD)) && isLoaded){
            state = ShooterState.STOP;
        }
        if((state.equals(ShooterState.AIM)) && isLoaded){
            isReady = true;
        }else{
            isReady = false;
        }
        runState();
        if (distMXP.getRange() < 8) {
            isLoaded = true;
        } else {
            isLoaded = false;
        }
        SmartDashboard.putBoolean("SHOOTER", isLoaded);
        SmartDashboard.putNumber("SHOOTER_ANGLE", angleEncoder.getPosition());
        SmartDashboard.putBoolean("SHOOTER_isReady", isReady);
        SmartDashboard.putNumber("r", r_ShooterEncoder.getVelocity());
        SmartDashboard.putNumber("l", l_ShooterEncoder.getVelocity());
    }

    private Shooter(){
        distMXP = new Rev2mDistanceSensor(Port.kMXP);
        l_ShooterMotor.restoreFactoryDefaults();
        r_ShooterMotor.restoreFactoryDefaults();
        l_ShooterMotor.setInverted(true);
        r_ShooterMotor.setIdleMode(IdleMode.kCoast);
        l_ShooterMotor.setIdleMode(IdleMode.kCoast);
        r_ShooterMotor.setClosedLoopRampRate(0.5);
        l_ShooterMotor.setClosedLoopRampRate(0.5);
        r_ShooterEncoder = r_ShooterMotor.getEncoder();
        l_ShooterEncoder = l_ShooterMotor.getEncoder();
        r_PIDController = r_ShooterMotor.getPIDController();     
        l_PIDController = l_ShooterMotor.getPIDController();  
        
        angleMotor1.restoreFactoryDefaults();
        angleMotor1.setInverted(true);
        angleMotor1.setIdleMode(IdleMode.kBrake);
        angleMotor2.follow(angleMotor1);
        angleEncoder = angleMotor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        angleEncoder.setPositionConversionFactor(360);

        FeederMotor.restoreFactoryDefaults();
        FeederMotor.setIdleMode(IdleMode.kBrake);

        anglePIDController = angleMotor1.getPIDController();
        anglePIDController.setFeedbackDevice(angleEncoder);

        double kP_P, kP_I, kP_D, kP_Iz, kP_FF, kP_MaxOutput, kP_MinOutput;
        // PID coefficients kPosition
        kP_P = 0.02; 
        kP_I = 0;
        kP_D = 0; 
        kP_Iz = 0; 
        kP_FF = 0; 
        kP_MaxOutput = 1; 
        kP_MinOutput = -1;

        // set PID coefficients
        anglePIDController.setP(kP_P);
        anglePIDController.setI(kP_I);
        anglePIDController.setD(kP_D);
        anglePIDController.setIZone(kP_Iz);
        anglePIDController.setFF(kP_FF);
        anglePIDController.setOutputRange(kP_MinOutput, kP_MaxOutput);

        double kV_P, kV_I, kV_D, kV_Iz, kV_FF, kV_MaxOutput, kV_MinOutput;
        // PID coefficients kVelocity
        kV_P = 0.00006; 
        kV_I = 0;
        kV_D = 0; 
        kV_Iz = 0; 
        kV_FF = 0.0002; 
        kV_MaxOutput = 1; 
        kV_MinOutput = -1;
        // set PID coefficients
        r_PIDController.setP(kV_P);
        r_PIDController.setI(kV_I);
        r_PIDController.setD(kV_D);
        r_PIDController.setIZone(kV_Iz);
        r_PIDController.setFF(kV_FF);
        r_PIDController.setOutputRange(kV_MinOutput, kV_MaxOutput);
        // set PID coefficients
        l_PIDController.setP(kV_P);
        l_PIDController.setI(kV_I);
        l_PIDController.setD(kV_D);
        l_PIDController.setIZone(kV_Iz);
        l_PIDController.setFF(kV_FF);
        l_PIDController.setOutputRange(kV_MinOutput, kV_MaxOutput);

        state = ShooterState.STOP;
    }

    public Command setState(ShooterState _state) {
        return Commands.runOnce(() -> state = _state);
    }
    
    public ShooterState getState() {
        return state;
    }

    private void runState(){
        Command currentShooterCommand = null;
        if(!state.equals(lastState)){
            SmartDashboard.putString("SHOOTER_STATE", state.name());
            switch(state){
                case SHOOT:
                    currentShooterCommand = launchShooter();
                    break;
                case AIM:
                    currentShooterCommand = prepareShooter();
                    break;
                case LOAD:
                    currentShooterCommand = loadShooter();
                    break;
                case STOP:
                    isReady = false;
                    currentShooterCommand = setShooterNeutral();
                    break;
                default:
                    isReady = false;
                    state = ShooterState.STOP;
                    break;
            }

            lastState = state;

            if (currentShooterCommand != null){
                currentShooterCommand.schedule();
            }
        }
    }

    //            /
    //           /
    //          /            robot's front
    //         /             ------->
    //        /
    //       /.  α
    // 180° /__)________________ 0°

    private Command setShooterAngle(){
        return Commands.runOnce(() -> {
            pivot(40);
        }, this);
    }

    private Command loadShooter(){
        return Commands.sequence(
            Commands.runOnce(() -> FeederMotor.set(0.8)),
            pivot(40)
        );
    }

    private Command setShooterNeutral(){
        return Commands.sequence(
            stopShooterWheels(),
            Commands.runOnce(() -> FeederMotor.stopMotor()),
            pivot(40)
        );
    }

    private Command launchShooter(){
        return Commands.sequence(
            startShooterWheels(5000, 5000),
            Commands.runOnce(() -> FeederMotor.set(1)),
            //Commands.runOnce(() -> intake.setState(IntakeState.IN)),
            pivot(60),
            Commands.waitSeconds(3),
            setShooterNeutral(),
            Commands.runOnce(() -> FeederMotor.stopMotor()),
            Commands.runOnce(() -> intake.setState(IntakeState.STOP)),
            Commands.runOnce(() -> state = ShooterState.STOP)
        );
    }

    private Command prepareShooter(){
        return Commands.sequence(
            Commands.runOnce(() -> FeederMotor.stopMotor()),
            pivot(60),
            startShooterWheels(5000, 5000)
        );
    }

    private Command startShooterWheels(double left_rpm, double right_rpm){
        return Commands.runOnce(() -> {
            l_PIDController.setReference(left_rpm , ControlType.kVelocity);
            r_PIDController.setReference(right_rpm , ControlType.kVelocity);
        }, this);
    }

    private Command stopShooterWheels(){
        return Commands.runOnce(() -> {
            r_ShooterMotor.disable();
            l_ShooterMotor.disable();
        }, this);
    }

    private Command pivot(double position){
        return Commands.run(() -> {
            anglePIDController.setReference(position, ControlType.kPosition);
        },this).until(() -> (Math.abs(position-angleEncoder.getPosition()) < 1.5));
    }
}