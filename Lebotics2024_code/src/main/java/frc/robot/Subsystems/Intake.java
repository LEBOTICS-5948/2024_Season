package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        STOP,
        DOWN,
        UP,
        CUSTOM
    }

    private IntakeState state = IntakeState.STOP;

    private final DigitalInput loadedLimitSwitch;
    private final DigitalInput zeroArmLimitSwitch;
    private final CANSparkMax rollerMotor;
    private final CANSparkMax angleMotor;

    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkMaxPIDController rollerPIDController;
    private final SparkMaxPIDController anglePIDController;
    private boolean loaded;
    private DutyCycleEncoder hexEncoder;

    private Intake(){
        
        hexEncoder = new DutyCycleEncoder(2);
        hexEncoder.setDistancePerRotation(360);
        hexEncoder.setPositionOffset(0.33);
        loadedLimitSwitch = new DigitalInput(0);
        zeroArmLimitSwitch = new DigitalInput(9);
        rollerMotor = new CANSparkMax(15, MotorType.kBrushless);
        angleMotor = new CANSparkMax(16, MotorType.kBrushless);
        angleMotor.setInverted(true);
        rollerEncoder = rollerMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        angleEncoder.setPositionConversionFactor(360/300);

        rollerPIDController = rollerMotor.getPIDController();
        anglePIDController = angleMotor.getPIDController();

        double kV_P, kV_I, kV_D, kV_Iz, kV_FF, kV_MaxOutput, kV_MinOutput;
        // PID coefficients kVelocity
        kV_P = 0.06; 
        kV_I = 0;
        kV_D = 0; 
        kV_Iz = 0; 
        kV_FF = 0.3; 
        kV_MaxOutput = 1; 
        kV_MinOutput = -1;
        // set PID coefficients
        rollerPIDController.setP(kV_P);
        rollerPIDController.setI(kV_I);
        rollerPIDController.setD(kV_D);
        rollerPIDController.setIZone(kV_Iz);
        rollerPIDController.setFF(kV_FF);
        rollerPIDController.setOutputRange(kV_MinOutput, kV_MaxOutput);

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
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("LS_roller", loadedLimitSwitch.get());
        SmartDashboard.putBoolean("LS_ARM", zeroArmLimitSwitch.get());
        SmartDashboard.putString("INTAKE_STATE", state.name());
        SmartDashboard.putNumber("encoder", hexEncoder.getDistance());
        SmartDashboard.putNumber("encoder1", angleEncoder.getPosition());
        SmartDashboard.putNumber("en", hexEncoder.get());
        if(zeroArmLimitSwitch.get()){
            angleEncoder.setPosition(0);
        }
    }

    private void down_take(){
        state = IntakeState.DOWN;
        if(loadedLimitSwitch.get()){
            stop_take();
        }
        anglePIDController.setReference(-201, ControlType.kPosition);
        if(angleEncoder.getPosition() < -150){
            rollerMotor.set(-0.6);
        }
    }

    private void up_take(){
        state = IntakeState.UP;
        if(zeroArmLimitSwitch.get()){
            angleMotor.stopMotor();
        }else{
            anglePIDController.setReference(0, ControlType.kPosition);
        }
    }

    private void stop_take(){
        state = IntakeState.STOP;
        rollerMotor.stopMotor();
        if(zeroArmLimitSwitch.get()){
            angleMotor.stopMotor();
        }else{
            anglePIDController.setReference(0, ControlType.kPosition);
        }
    }

    private void CUSTOM(){
        state = IntakeState.CUSTOM;
        anglePIDController.setReference(-90, ControlType.kPosition);
    }

    public void runState(){
        angleEncoder.setPosition(hexEncoder.getDistance());
        if(state == IntakeState.STOP){
            stop_take();
        }else if(state == IntakeState.DOWN){
            down_take();
        }else if(state == IntakeState.UP){
            up_take();
        }else if(state == IntakeState.CUSTOM){
            CUSTOM();
        }
    }

    public void setState(IntakeState new_state){
        state = new_state;
    }
    public void shoot(){
        if(state == IntakeState.CUSTOM){
            rollerMotor.set(1);
        }
    }
}
