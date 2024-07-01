package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private final CANSparkMax angleMotor;

    private final RelativeEncoder angleEncoder;

    private final SparkMaxPIDController anglePIDController;

    public boolean isIntaking = false;
    public boolean isLoaded = false;

    private Intake(){
        loadedLimitSwitch = new DigitalInput(1);
        zeroArmLimitSwitch = new DigitalInput(9);
        rollerMotor = new CANSparkMax(15, MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setIdleMode(IdleMode.kCoast);
        angleMotor = new CANSparkMax(16, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(true);
        angleMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder = angleMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        angleEncoder.setPositionConversionFactor(360);

        anglePIDController = angleMotor.getPIDController();
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

        state = IntakeState.IDLE;
    }

    @Override
    public void periodic(){
        isLoaded = !loadedLimitSwitch.get();
        if(zeroArmLimitSwitch.get()){ angleEncoder.setPosition(0); }
        if(isIntaking && isLoaded){ state = IntakeState.STOP; }
        runState();
        SmartDashboard.putNumber("INTAKE_ANGLE", angleEncoder.getPosition());
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
                case DOWN:
                    isIntaking = true;
                    currentIntakeCommand = down_take();
                    break;
                case UP:
                    isIntaking = true;
                    currentIntakeCommand = up_take();
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
            angleMotor.disable();
        },this);
    }

    private Command down_take(){
        return Commands.sequence(
            pivot(-192).andThen(stopArm()).andThen(rollers(-0.8))
        );
    }

    private Command up_take(){
        return Commands.sequence(
            pivot(-60).andThen(stopArm()).andThen(rollers(-0.6))
        );
    }

    private Command stop_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.stopMotor()),
            pivot(0).andThen(stopArm())
        );
    }
    private Command hold_take(){
        return Commands.sequence(
            Commands.runOnce(() -> rollerMotor.set(-0.08)),
            pivot(0).andThen(stopArm())
        );
    }

    private Command amp_take(){
        return Commands.sequence(
            rollers(-0.06),
            pivot(-73).andThen(
                Commands.runOnce(() -> angleMotor.stopMotor()),
                Commands.waitSeconds(0.2),rollers(0.9),
                Commands.waitSeconds(0.4),setState(IntakeState.STOP)
            )
        );
    }

    private Command speaker_take(){
        return Commands.sequence(
            pivot(0),
            rollers(1),
            Commands.waitSeconds(1),
            setState(IntakeState.STOP)
        );
    }

    private Command pivot(double position){
        return Commands.run(() -> {
            anglePIDController.setReference(position, ControlType.kPosition);
        },this).until(() -> (Math.abs(position-angleEncoder.getPosition()) < 1.5));
    }

    private Command rollers(double speed){
        return Commands.runOnce(() -> rollerMotor.set(speed),this);
    }

    private Command stopArm(){
        return Commands.runOnce(() -> angleMotor.stopMotor(),this);
    }
}
