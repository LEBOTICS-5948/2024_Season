package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

// import edu.wpi.first.math.geometry.Transform2d; Library not used.

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
        AUTO,
        STOP,
    }

    private final SwerveModule sm_backLeft = new SwerveModule(7, 8, 3,false); // 1
    private final SwerveModule sm_frontLeft = new SwerveModule(5, 6, 6,false); // 2
    private final SwerveModule sm_frontRight = new SwerveModule(3, 4, 9,true); // 3
    private final SwerveModule sm_backRight = new SwerveModule(1, 2, 12,true); // 4

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
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private Field2d field = new Field2d();
    private SwerveDriveOdometry m_odometry;
    private Pose2d pose;

    private DoubleSupplier translationX, translationY, rotationOmega, omegaOverride;

    private Double DEADBAND = 0d;

    private Boolean FieldRelativeTeleop = true, useOmegaOverride = false;

    private DriveTrainState state = DriveTrainState.IDLE, lastState;

    

    private DriveTrain() {
        gyro.reset();
        pose = new Pose2d();
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, gyro.getRotation2d(),
            getSwerveModulePositions(), new Pose2d(0, 0, new Rotation2d()));

            AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(4, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
                        SwerveConstants.DriveMaxSpeed, // Max module speed, in m/s
                        0.34, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.equals(DriverStation.Alliance.Red);
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData(field);
        SmartDashboard.putBoolean("FieldRelativeTeleop", FieldRelativeTeleop);
        SmartDashboard.putNumber("DeadBand_Drift", DEADBAND);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d p) {
        m_odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), p);
    }

    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            sm_backLeft.getState(),
            sm_frontLeft.getState(),
            sm_frontRight.getState(),
            sm_backRight.getState()
        };
        return states;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        targetStates[0].angle = targetStates[0].angle.plus(new Rotation2d(Math.PI/2));
        targetStates[1].angle = targetStates[1].angle.plus(new Rotation2d(Math.PI/2));
        targetStates[2].angle = targetStates[2].angle.plus(new Rotation2d(Math.PI/2));
        targetStates[3].angle = targetStates[3].angle.plus(new Rotation2d(Math.PI/2));
        setModuleStates(targetStates);
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
        if(!state.equals(DriveTrainState.AUTO)){
            SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
            setModuleStates(swerveModuleStates);
        }
        pose = m_odometry.update(gyro.getRotation2d(), getSwerveModulePositions());
        field.setRobotPose(pose);
        SmartDashboard.putNumber("pose_x", pose.getX());
        SmartDashboard.putNumber("pose_y", pose.getY());
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {
            sm_backLeft.getModulePosition(),
            sm_frontLeft.getModulePosition(),
            sm_frontRight.getModulePosition(),
            sm_backRight.getModulePosition()
        };
        return positions;
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
                    currentDriveCommand = teleopDrive()
                        .until(() -> !isJoystickInputPresent())
                        .finallyDo((interrupted) -> {
                            if (!interrupted)
                                setState(DriveTrainState.IDLE).schedule();
                        });
                    break;
                case AUTO:
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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, gyro.getRotation2d())
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
            double rot = (FieldRelativeTeleop&&useOmegaOverride) ? omegaOverride.getAsDouble(): MathUtil.applyDeadband(rotationOmega.getAsDouble()*1.8, DEADBAND);

            drive(xSpeed, ySpeed, rot, FieldRelativeTeleop);
        });
    } 
    
    public void setFieldTrajectory(Trajectory t) {
        field.getObject("traj").setTrajectory(t);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        sm_backLeft.setDesiredState(desiredStates[0]);
        sm_frontLeft.setDesiredState(desiredStates[1]);
        sm_frontRight.setDesiredState(desiredStates[2]);
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