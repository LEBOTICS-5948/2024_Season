package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private static Shooter m_instance = null;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter();
        }
        return m_instance;
    }

    private final CANSparkMax r_ShooterMotor = new CANSparkMax(17, MotorType.kBrushless);
    private final CANSparkMax l_ShooterMotor = new CANSparkMax(18, MotorType.kBrushless);

    private final RelativeEncoder r_ShooterEncoder;
    private final RelativeEncoder l_ShooterEncoder;

    private final SparkMaxPIDController r_PIDController;
    private final SparkMaxPIDController l_PIDController;

    private boolean isReady = false;

    @Override
    public void periodic(){
        SmartDashboard.putNumber("r_shooter", r_ShooterEncoder.getVelocity());
        SmartDashboard.putNumber("l_shooter", l_ShooterEncoder.getVelocity());
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
        kV_FF = 0.00017; 
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
    }

    public void startShooter(){
        r_PIDController.setReference(6000 , ControlType.kVelocity);
        l_PIDController.setReference(6000 , ControlType.kVelocity);
    }

    public void stopShooter(){
        r_ShooterMotor.disable();
        l_ShooterMotor.disable();
    }
}