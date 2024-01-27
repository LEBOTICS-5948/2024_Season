package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class DriveTrain extends SubsystemBase{

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

    //private final ADIS16448_IMU gyro = new ADIS16448_IMU();
    private final AHRS gyro = new AHRS(Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = SwerveConstants.m_kinematics;

    private static DriveTrain m_instance = null;

    public static DriveTrain getInstance() {
        if (m_instance == null) {
            m_instance = new DriveTrain();
        }
        return m_instance;
    }

    private DriveTrain() {
        gyro.reset();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("gyro", gyro.getYaw());
    }

    /**
    * Method to drive the robot using joystick info.
    *
    * @param xSpeed Speed of the robot in the x direction (forward).
    * @param ySpeed Speed of the robot in the y direction (sideways).
    * @param rot Angular rate of the robot.
    * @param fieldRelative Whether the provided x and y speeds are relative to the field.
    */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(Math.toRadians(-gyro.getYaw())))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(swerveModuleStates);
    } 

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        sm_backLeft.setDesiredState(desiredStates[0]);
        sm_frontLeft.setDesiredState(desiredStates[1]);
        desiredStates[2].speedMetersPerSecond *= -1; 
        sm_frontRight.setDesiredState(desiredStates[2]);
        desiredStates[3].speedMetersPerSecond *= -1; 
        sm_backRight.setDesiredState(desiredStates[3]);
    }
}
