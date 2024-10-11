package frc.robot.Subsystems;

// Libraries not used.
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxAlternateEncoder;
//import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake m_instance = null;

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    public enum IntakeState {
        IDLE,
        STOP,
        HOLD,
        DOWN,
        UP,
        AMP,
        SPEAKER,
    }

    private IntakeState state, lastState;

    private final DigitalInput loadedLimitSwitch;
    private final DigitalInput zeroArmLimitSwitch;
    private final CANSparkMax rollerMotor;

    public boolean isIntaking = false;
    public boolean isLoaded = false;

    private Intake(){
        loadedLimitSwitch = new DigitalInput(1);
        zeroArmLimitSwitch = new DigitalInput(9);
        rollerMotor = new CANSparkMax(11, MotorType.kBrushless); 
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setIdleMode(IdleMode.kCoast);

        /*
        // PID coefficients kPosition
        double kP_P, kP_I, kP_D, kP_Iz, kP_FF, kP_MaxOutput, kP_MinOutput;
        kP_P = 0.02; 
        kP_I = 0;
        kP_D = 0; 
        kP_Iz = 0; 
        kP_FF = 0; 
        kP_MaxOutput = 1; 
        kP_MinOutput = -1;
        */

        // set PID coefficients
        state = IntakeState.IDLE;
    }

    @Override
    public void periodic(){
        isLoaded = !loadedLimitSwitch.get();
        if(isIntaking && isLoaded){ state = IntakeState.STOP; }
        runState();
        SmartDashboard.putBoolean("LS_ROLLER", isLoaded);
        SmartDashboard.putBoolean("LS_ARM", zeroArmLimitSwitch.get());
        SmartDashboard.putBoolean("isIntaking", isIntaking);
    }

    public Command setState(IntakeState _state) {
        return Commands.runOnce(() -> state = _state);
    }
    
    public IntakeState getState() {
        return state;
    }

    private void runState(){
        Command currentIntakeCommand = null;
        if(!state.equals(lastState)){
            SmartDashboard.putString("INTAKE_STATE", state.name());
            switch(state){
                case IDLE:
                    isIntaking = false;
                    currentIntakeCommand = idle_take();
                    break;
                case STOP:
                    currentIntakeCommand = stop_take().finallyDo((i) -> isIntaking = false);
                    break;
                case HOLD:
                    currentIntakeCommand = hold_take().finallyDo((i) -> isIntaking = false);
                    break;
                case AMP:
                    isIntaking = false;
                    currentIntakeCommand = amp_take();
                    break;
                case SPEAKER:
                    isIntaking = false;
                    currentIntakeCommand = speaker_take();
                    break;
                default:
                    state = IntakeState.STOP;
                    break;
            }

            lastState = state;

            if (currentIntakeCommand != null){
                currentIntakeCommand.schedule();
            }
        }
    }

    private Command idle_take(){
        return Commands.runOnce(() -> {
            rollerMotor.disable();
        },this);
    }

    private Command stop_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.stopMotor())
        );
    }
    private Command hold_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.set(-0.08))
        );
    }

    private Command amp_take(){
        return Commands.sequence(
            rollers(-0.06),
            Commands.waitSeconds(0.2),rollers(0.9),
            Commands.waitSeconds(0.4),setState(IntakeState.STOP)
        );
    }

    private Command speaker_take(){
        return Commands.sequence(
            rollers(1),
            Commands.waitSeconds(1),
            setState(IntakeState.STOP)
        );
    }

    private Command rollers(double speed){
        return Commands.runOnce(() -> rollerMotor.set(speed),this);
    }
}