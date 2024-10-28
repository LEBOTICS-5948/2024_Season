package frc.robot.Subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.Subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight extends SubsystemBase {
    private final LimelightHelpers limelight;

    private final DriveTrain swerveDrive;

    public Limelight() {
        // Detecta el color de la alianza usando DriverStation
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        String allianceColor = alliance == Alliance.Red ? "red" : "blue";

        // Inicializa LimelightHelpers con el color de la alianza detectado
        this.limelight = new LimelightHelpers(allianceColor);
        this.swerveDrive = DriveTrain.getInstance();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("IsTargeting", limelight.isTargeting());

        // Verifica si la pose es válida
        if (limelight.isTargeting()) {
            double[] botPose = limelight.getAllianceBotPose();

            double x = botPose[0]; // Posición X en metros
            double y = botPose[1]; // Posición Y en metros
            double yaw = botPose[5]; // Rotación en grados

            Rotation2d rotation = Rotation2d.fromDegrees(yaw);

            swerveDrive.updateOdometry(new Pose2d(x,y,rotation));

            SmartDashboard.putNumber("Robot X", x);
            SmartDashboard.putNumber("Robot Y", y);
            SmartDashboard.putNumber("Robot Yaw", yaw);
        }
    }

    // Método para obtener la posición actual del robot desde la odometría
    public Pose2d getCurrentPose() {
        return swerveDrive.getPose();
    }
}
