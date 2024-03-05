package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;


public class SwerveModule{

    private final double WheelDiameter = SwerveConstants.WheelDiameter;
    private final double DriveMaxSpeed = SwerveConstants.DriveMaxSpeed;
    private final double DriveMaxAcc = SwerveConstants.DriveMaxAcc;
    private final double TurningMaxAcc = SwerveConstants.TurningMaxAcc;
    private final double TurningGearRatio = SwerveConstants.TurningGearRatio;
    private final double DriveGearRatio = SwerveConstants.DriveGearRatio;

    private final CANSparkMax turningMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;
    private final CANCoder directionalEncoder;

    private final SparkMaxPIDController turningPIDController;
    private final SparkMaxPIDController drivePIDController;

    private SwerveModuleState state = new SwerveModuleState();

    /**
     * Construct a motor with a drive motor, tunring motor, directional encoder
     * 
     * @param turningMotorID CAN ID for turningMotor
     * @param driveMotorID CAN ID for driveMotor
     * @param directionalEncoderID CAN ID for directionalEncoder
     * 
     */
    public SwerveModule(int turningMotorID,int driveMotorID,int directionalEncoderID, boolean isInverted){
        
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        directionalEncoder = new CANCoder(directionalEncoderID, SwerveConstants.CANbus);

        turningMotor.restoreFactoryDefaults();
        turningMotor.setClosedLoopRampRate(TurningMaxAcc);
        turningMotor.setInverted(true);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setClosedLoopRampRate(DriveMaxAcc);
        driveMotor.setInverted(isInverted);

        turnEncoder = turningMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(360/TurningGearRatio);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Math.PI*WheelDiameter/DriveGearRatio);
        driveEncoder.setVelocityConversionFactor(Math.PI*WheelDiameter/(60*DriveGearRatio));
        driveEncoder.setPosition(0);

        turningPIDController = turningMotor.getPIDController();
        drivePIDController = driveMotor.getPIDController();

        double kP_P, kP_I, kP_D, kP_Iz, kP_FF, kP_MaxOutput, kP_MinOutput;
        // PID coefficients kPosition
        kP_P = 0.006; 
        kP_I = 0;
        kP_D = 0; 
        kP_Iz = 0; 
        kP_FF = 0; 
        kP_MaxOutput = 1; 
        kP_MinOutput = -1;

        // set PID coefficients
        turningPIDController.setP(kP_P);
        turningPIDController.setI(kP_I);
        turningPIDController.setD(kP_D);
        turningPIDController.setIZone(kP_Iz);
        turningPIDController.setFF(kP_FF);
        turningPIDController.setOutputRange(kP_MinOutput, kP_MaxOutput);

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
        drivePIDController.setP(kV_P);
        drivePIDController.setI(kV_I);
        drivePIDController.setD(kV_D);
        drivePIDController.setIZone(kV_Iz);
        drivePIDController.setFF(kV_FF);
        drivePIDController.setOutputRange(kV_MinOutput, kV_MaxOutput);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        var absolutePosition = directionalEncoder.getAbsolutePosition();

        /* if(new Rotation2d(Math.toRadians(absolutePosition)) != new Rotation2d(Math.toRadians(directionalEncoder.getPosition()))){
            directionalEncoder.setPosition()
        } */
        // Optimize the reference state to avoid spinning further than 90 degrees
        state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(absolutePosition)));

        // Set reference for the turning PID controller
        
        turnEncoder.setPosition(directionalEncoder.getPosition());
        double setPoint = turnEncoder.getPosition() + optimizeOptimize(state.angle.getDegrees(), absolutePosition);
        turningPIDController.setReference(setPoint, ControlType.kPosition);

        // Set reference for the drive PID controller

        drivePIDController.setReference(state.speedMetersPerSecond*DriveMaxSpeed, ControlType.kVelocity);
        
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition()*1.23, new Rotation2d(Math.toRadians(directionalEncoder.getAbsolutePosition()-90)));
        //driveEncoder.setPosition(0);
    }

    private double optimizeOptimize(double da, double Aa){
        
        double a = Math.abs(Aa-da);
        if(da < Aa && a < 180){ a = -a;}else
        if(!(a < 180)){
            a = 360-a;
            if(da > Aa){ a = -a;}
        } 
        return a;
    }

    public SwerveModuleState getState(){
        return state;
    }

    
}
