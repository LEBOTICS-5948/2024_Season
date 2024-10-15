package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftingArmsConstants;

public class LiftingArms extends SubsystemBase{

    private static LiftingArms m_instance = null;

    public static LiftingArms getInstance() {
        if (m_instance == null) {
            m_instance = new LiftingArms();
        }
        return m_instance;
    }

    public enum LiftingArmsState{
        UP,
        DOWN,
        STOP
    }

    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    //private RelativeEncoder leftArmEncoder;
    //private RelativeEncoder rightArmEncoder;

    private LiftingArmsState state, lastState;
    

    private LiftingArms() {
        leftArmMotor = new CANSparkMax(LiftingArmsConstants.LeftArmID, MotorType.kBrushless);
        leftArmMotor.restoreFactoryDefaults();
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setOpenLoopRampRate(0.4);

        rightArmMotor = new CANSparkMax(LiftingArmsConstants.RightArmID, MotorType.kBrushless);
        rightArmMotor.restoreFactoryDefaults();
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setOpenLoopRampRate(0.4);

        //leftArmEncoder = leftArmMotor.getEncoder();
        //rightArmEncoder = rightArmMotor.getEncoder();

        leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LiftingArmsConstants.SoftLimit);
        leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -LiftingArmsConstants.SoftLimit); 

        state = LiftingArmsState.STOP;
        
    }

    @Override
    public void periodic(){
        runState();
    }

    public Command setState(LiftingArmsState _state) {
        return Commands.runOnce(() -> state = _state);
    }
    
    public LiftingArmsState getState() {
        return state;
    }

    private void runState(){
        Command currentArmsCommand = null;
        if(!state.equals(lastState)){
            SmartDashboard.putString("LIFTING_ARMS_STATE", state.name());
            switch(state){
                case STOP:
                    currentArmsCommand = stopArms();
                    break;
                case DOWN:
                    currentArmsCommand = retractArms();
                    break;
                case UP:
                    currentArmsCommand = extendArms();
                    break;
                default:
                    state = LiftingArmsState.STOP;
                    break;
            }

            lastState = state;

            if (currentArmsCommand != null){
                currentArmsCommand.schedule();
            }
        }
    }

    public Command retractArms(){
        return Commands.runOnce(() -> {
            leftArmMotor.set(-1);
            rightArmMotor.set(1);
        },this);
    }

    public Command extendArms(){
        return Commands.runOnce(() -> {
            leftArmMotor.set(1);
            rightArmMotor.set(-1);
        },this);
    }

    public Command stopArms(){
        return Commands.runOnce(() -> {
            leftArmMotor.stopMotor();
            rightArmMotor.stopMotor();
        },this);
    }
}