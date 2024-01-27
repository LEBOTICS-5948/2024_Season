package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftingArmsConstants;

public class LiftingArms extends SubsystemBase{
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private RelativeEncoder leftArmEncoder;
    private RelativeEncoder rightArmEncoder;
    

    private static LiftingArms m_instance = null;

    public static LiftingArms getInstance() {
        if (m_instance == null) {
            m_instance = new LiftingArms();
        }
        return m_instance;
    }

    private LiftingArms() {
        leftArmMotor = new CANSparkMax(LiftingArmsConstants.leftArmID, MotorType.kBrushless);
        leftArmMotor.restoreFactoryDefaults();
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setOpenLoopRampRate(0.4);

        rightArmMotor = new CANSparkMax(LiftingArmsConstants.rightArmID, MotorType.kBrushless);
        rightArmMotor.restoreFactoryDefaults();
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setOpenLoopRampRate(0.4);

        leftArmEncoder = leftArmMotor.getEncoder();
        rightArmEncoder = rightArmMotor.getEncoder();

        leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -93.5f);

        rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 93.5f);
        rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); 
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("leftPos", leftArmEncoder.getPosition()/-93.5);
        SmartDashboard.putNumber("rightPos", rightArmEncoder.getPosition()/93.5);
    }

    public void retractArms(){
        leftArmMotor.set(0.5);
        rightArmMotor.set(-0.5);
    }

    public void extendArms(){
        leftArmMotor.set(-0.5);
        rightArmMotor.set(0.5);
    }

    public void stopArms(){
        leftArmMotor.stopMotor();
        rightArmMotor.stopMotor();
    }
}
