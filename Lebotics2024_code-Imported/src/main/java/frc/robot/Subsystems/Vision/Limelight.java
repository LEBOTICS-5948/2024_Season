package frc.robot.Subsystems.Vision;

import java.util.function.Supplier;

import org.opencv.core.Algorithm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.Subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight extends SubsystemBase {
    private static Limelight instance; // Única instancia de Limelight
    
    private final LimelightHelpers limelight;
    private final DriveTrain swerveDrive;
    private final Pose2d blue_speaker = new Pose2d(0.0, 5.55, new Rotation2d(0));
    private final Pose2d red_speaker = new Pose2d(16.54, 5.55, new Rotation2d(0));
    private final Alliance alliance;

    private Limelight(DriveTrain driveTrain) {
        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        String allianceColor = alliance == Alliance.Red ? "red" : "blue";
        
        this.limelight = new LimelightHelpers("blue");
        this.swerveDrive = driveTrain;

        SmartDashboard.putBoolean("FixedShooterAngle", false);
        SmartDashboard.putNumber("ShooterAngle", 30);
    }

    public static synchronized Limelight getInstance(DriveTrain driveTrain) {
        if (instance == null) {
            instance = new Limelight(driveTrain);
        }
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsTargeting", limelight.isTargeting());
        SmartDashboard.putNumber("angleToTarget", angloToTarget());
        SmartDashboard.putNumber("distanceToTarget", getDistanceToTarget());

        if (limelight.isTargeting()) {
            double[] botPose = limelight.getAllianceBotPose();
            double x = botPose[0];
            double y = botPose[1];
            double yaw = botPose[5];

            Rotation2d rotation = Rotation2d.fromDegrees(yaw);
            swerveDrive.updateOdometry(new Pose2d(x, y, rotation), rotation);
        }
    }

    public double getShooterAngle() {
        
        Boolean FSA = SmartDashboard.getBoolean("FixedShooterAngle", false);
        if(FSA) {
            return SmartDashboard.getNumber("ShooterAngle", 0);
        }else{
            getDistanceToTarget();
            return 0;
        }
    }

    public double angloToTarget() {
        Pose2d target = alliance == Alliance.Red ? red_speaker : blue_speaker;
        Pose2d robotPose = swerveDrive.getPose();
        
        double angle = calculateTargetAngle(robotPose, target).getDegrees();
        if(alliance == Alliance.Blue) {
            return 360 - (angle < 0 ? angle + 360 : angle);
        }else{
            return 360 - (((angle < 0 ? angle + 360 : angle) + 180) % 360);
        }
        
    }

    private Rotation2d calculateTargetAngle(Pose2d robotPose, Pose2d target) {
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();
        
        double angleToTargetRadians = Math.atan2(deltaY, deltaX);
        return Rotation2d.fromRadians(angleToTargetRadians);
    }

    private double getDistanceToTarget() {
        Pose2d target = alliance == Alliance.Red ? red_speaker : blue_speaker;
        Pose2d robotPose = swerveDrive.getPose();

        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();
        
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    

}
