package frc.lib.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {
    private final NetworkTable limelightTable;
    private final String allianceColor;

    public LimelightHelpers(String allianceColor) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.allianceColor = allianceColor.toLowerCase(); // "blue" o "red"
    }

    // Verifica si la limelight está enfocando un objetivo
    public boolean isTargeting() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    // Obtiene la pose del robot en el campo ajustada por alianza (botpose_wpiblue o botpose_wpired)
    public double[] getAllianceBotPose() {
        String poseKey = allianceColor.equals("blue") ? "botpose_wpiblue" : "botpose_wpired";
        double[] botPose = limelightTable.getEntry(poseKey).getDoubleArray(new double[6]);

        if (botPose.length < 6) {
            System.out.println("Error: No se pudo obtener la pose completa del robot.");
            return new double[6];
        }

        return botPose;
    }

    // Obtiene la distancia aproximada al objetivo
    public double getTargetDistance() {
        return limelightTable.getEntry("ta").getDouble(0);
    }
}
