package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class DriveTrain extends SubsystemBase{

    private static DriveTrain m_instance = null;

    public static DriveTrain getInstance() {
        if (m_instance == null) {
            m_instance = new DriveTrain();
        }
        return m_instance;
    }

    public enum DriveTrainState{
        JOYSTICKS,
        IDLE,
        TRAYECTORY
    }

    private final SwerveModule sm_backLeft = new SwerveModule(1, 2, 3); // 1
    private final SwerveModule sm_frontLeft = new SwerveModule(4, 5, 6); // 2
    private final SwerveModule sm_frontRight = new SwerveModule(7, 8, 9); // 3
    private final SwerveModule sm_backRight = new SwerveModule(10, 11, 12); // 4

    //    .---.         .---.
    //    | 2 |▩▩▩▩▩▩| 3 |
    //    '---'         '---'
    //      ▩     /\     ▩
    //      ▩    /||\    ▩
    //      ▩     ||     ▩
    //    .---.         .---.
    //    | 1 |▩▩▩▩▩▩| 4 |
    //    '---'         '---'

    private final AHRS gyro = new AHRS(Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = SwerveConstants.Kinematics;

    private DoubleSupplier translationX, translationY, rotationOmega, omegaOverride;

    private Double DEADBAND = 0d;

    private Boolean FieldRelativeTeleop = true, useOmegaOverride = false;

    private DriveTrainState state = DriveTrainState.IDLE, lastState;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private DriveTrain() {
        gyro.reset();
        SmartDashboard.putBoolean("FieldRelativeTeleop", FieldRelativeTeleop);
        SmartDashboard.putNumber("DeadBand_Drift", DEADBAND);
    }

    @Override
    public void periodic(){
        boolean FRT = SmartDashboard.getBoolean("FieldRelativeTeleop", true);
        if(FRT != FieldRelativeTeleop){ FieldRelativeTeleop = FRT;}
        double DZ = SmartDashboard.getNumber("DeadBand_Drift", 0);
        if(DZ != DEADBAND){ DEADBAND = DZ;}
        SmartDashboard.putBoolean("useOmegaOverride", useOmegaOverride);
        SmartDashboard.putBoolean("joystickInput", isJoystickInputPresent());
        SmartDashboard.putNumber("gyro", gyro.getYaw());
        runState();
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public void setJoystickSuppliers(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput){
        translationX = xInput;
        translationY = yInput;
        rotationOmega = omegaInput;
    }

    private boolean isJoystickInputPresent() {
        return (!Stream
            .of(translationX.getAsDouble(), translationY.getAsDouble(),
                rotationOmega.getAsDouble())
            .filter((input) -> MathUtil.applyDeadband(input, DEADBAND) != 0)
            .collect(Collectors.toList()).isEmpty())||useOmegaOverride;
    }

    public Command setState(DriveTrainState _state) {
        return Commands.runOnce(() -> state = _state);
    }
    
    public DriveTrainState getState() {
        return state;
    }

    private void runState(){
        Command currentDriveCommand = null;
        if(!state.equals(lastState)){
            SmartDashboard.putString("DRIVE_STATE", state.name());
            switch(state){
                case IDLE:
                    currentDriveCommand = idleDrive().repeatedly().until(() -> isJoystickInputPresent())
                        .finallyDo((interrupted) -> {
                            if (!interrupted)
                                setState(DriveTrainState.JOYSTICKS).schedule();
                        });
                    break;
                case JOYSTICKS:
                    System.out.println("hola negro");
                    currentDriveCommand = teleopDrive()
                        .until(() -> !isJoystickInputPresent())
                        .finallyDo((interrupted) -> {
                            if (!interrupted)
                                setState(DriveTrainState.IDLE).schedule();
                        });
                    break;
                case TRAYECTORY:
                    state = DriveTrainState.IDLE;
                    break;
                default:
                    state = DriveTrainState.IDLE;
                    break;
            }

            lastState = state;

            if (currentDriveCommand != null){
                currentDriveCommand.schedule();
            }
        }
    }

    private void drive(double x, double y, double omega, boolean fieldRelative){
        chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, new Rotation2d(Math.toRadians(-gyro.getYaw())))
            : new ChassisSpeeds(x, y, omega);
    }

    private Command idleDrive(){
        return Commands.runOnce(
        () -> chassisSpeeds = new ChassisSpeeds(), this);
    }

    private Command teleopDrive() {
        return new RunCommand(() -> {
            double xSpeed = MathUtil.applyDeadband(translationX.getAsDouble(), DEADBAND);
            double ySpeed = MathUtil.applyDeadband(translationY.getAsDouble(), DEADBAND);
            double rot = (FieldRelativeTeleop&&useOmegaOverride) ? omegaOverride.getAsDouble(): MathUtil.applyDeadband(rotationOmega.getAsDouble(), DEADBAND);

            drive(xSpeed, ySpeed, rot, FieldRelativeTeleop);
        });
    } 

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        sm_backLeft.setDesiredState(desiredStates[0]);
        sm_frontLeft.setDesiredState(desiredStates[1]);
        desiredStates[2].speedMetersPerSecond *= -1; 
        sm_frontRight.setDesiredState(desiredStates[2]);
        desiredStates[3].speedMetersPerSecond *= -1; 
        sm_backRight.setDesiredState(desiredStates[3]);
    }

    public Command setRotationSuplier(boolean isFixed,Double a){
        return 
            Commands.runOnce(() -> {
                omegaOverride = () -> !isFixed ? null : getFixedOmega(a); 
                useOmegaOverride = isFixed;
                lastState = null;
            }, this);
        
        
        
    }

    private double getFixedOmega(double targetAngle) {

        try (PIDController omegaPID = new PIDController(0.019, 0.0, 0.0)) {
            omegaPID.enableContinuousInput(-180, 180);
            omegaPID.setTolerance(1);
            
            double error = targetAngle - gyro.getYaw();
            error = MathUtil.inputModulus(error, -180, 180);
            
            return omegaPID.calculate(error);
        }
    }
}
