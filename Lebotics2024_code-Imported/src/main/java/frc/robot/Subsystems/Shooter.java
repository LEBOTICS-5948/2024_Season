package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private static Shooter m_instance = null;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter();
        }
        return m_instance;
    }

    public enum ShooterState{
        START_HIGHT,
        START_LOW,
        STOP
    }

    private final CANSparkMax r_ShooterMotor = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax l_ShooterMotor = new CANSparkMax(13, MotorType.kBrushless);

    private final RelativeEncoder r_ShooterEncoder;
    private final RelativeEncoder l_ShooterEncoder;

    private final SparkPIDController r_PIDController;
    private final SparkPIDController l_PIDController;

    private ShooterState state, lastState;

    public boolean isReady = false;

    private double targetRPM; //shooter_speed

    @Override
    public void periodic(){
        runState();
        /* if((!state.equals(ShooterState.STOP)) && r_ShooterEncoder.getVelocity()>targetRPM && l_ShooterEncoder.getVelocity()>targetRPM){
            isReady = true;
        } */
        if((!state.equals(ShooterState.STOP)) && r_ShooterEncoder.getVelocity()>targetRPM){
            isReady = true;
        }
        SmartDashboard.putBoolean("SHOOTER_isReady", isReady);
        SmartDashboard.putNumber("r", r_ShooterEncoder.getVelocity());
        SmartDashboard.putNumber("l", l_ShooterEncoder.getVelocity());
    }

    private Shooter(){
        r_ShooterMotor.setInverted(true);
        r_ShooterMotor.setClosedLoopRampRate(0.5);
        l_ShooterMotor.setClosedLoopRampRate(0.5);
        r_ShooterEncoder = r_ShooterMotor.getEncoder();
        l_ShooterEncoder = l_ShooterMotor.getEncoder();
        r_PIDController = r_ShooterMotor.getPIDController();     
        l_PIDController = l_ShooterMotor.getPIDController();     

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
                case START_HIGHT:
                    currentShooterCommand = startShooter(4400);
                    break;
                case START_LOW:
                    currentShooterCommand = startShooter(2000);
                    break;
                case STOP:
                    isReady = false;
                    currentShooterCommand = stopShooter();
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

    private Command startShooter(double speed){
        return Commands.runOnce(() -> {
            targetRPM = speed-100;
            r_PIDController.setReference(speed , ControlType.kVelocity);
            l_PIDController.setReference(speed , ControlType.kVelocity);
        }, this);
    }

    private Command stopShooter(){
        return Commands.runOnce(() -> {
            r_ShooterMotor.disable();
            l_ShooterMotor.disable();
        }, this);
    }
}