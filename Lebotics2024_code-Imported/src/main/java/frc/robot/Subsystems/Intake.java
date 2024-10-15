package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake m_instance = null;
    private Rev2mDistanceSensor distOnboard;

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    public enum IntakeState {
        STOP,
        IN,
        OUT,
        TAKE,
    }

    private IntakeState state, lastState;

    private final CANSparkMax rollerMotor;

    public boolean isIntaking = false;
    public boolean isLoaded = false;

    private Intake(){
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        rollerMotor = new CANSparkMax(11, MotorType.kBrushless); 
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setIdleMode(IdleMode.kCoast);
        // set initial state
        state = IntakeState.STOP;
    }

    @Override
    public void periodic(){
        //if(isIntaking && isLoaded){ state = IntakeState.STOP; }
        runState();
        if (distOnboard.getRange() < 10) {
            isLoaded = true;
        } else {
            isLoaded = false;
        }
        SmartDashboard.putBoolean("INTAKE", isLoaded);
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
                case STOP:
                    currentIntakeCommand = STOP_take().finallyDo((i) -> isIntaking = false);
                    break;
                case IN:
                    isIntaking = true;
                    currentIntakeCommand = IN_take();
                    break;
                case OUT:
                    isIntaking = true;
                    currentIntakeCommand = OUT_take();
                    break;
                case TAKE:
                    isIntaking = false;
                    currentIntakeCommand = TAKE_take();
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

    private Command IN_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.set(0.8))
        );
    }

    private Command OUT_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.set(-0.8))
        );
    }

    private Command STOP_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.stopMotor())
        );
    }
    private Command TAKE_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.set(-0.08))
        );
    }

    private Command rollers(double speed){
        return Commands.runOnce(() -> rollerMotor.set(speed),this);
    }
}