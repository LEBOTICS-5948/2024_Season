package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

// Recomiendo mantener estas funciones declaradas para disminuir la memoria que se destina en el robot.
/* 
Librerias que se puedan utilizar a futuro.
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.DriverStationJNI;
import javax.xml.crypto.Data;
import edu.wpi.first.util.datalog.DataLogReader;

Librerias siguientes a implementar.
import edu.wpi.first.wpilibj.PowerDistribution; // Saber el estado de la bateria y comunicarle al driver si es necesario cambiarla.
*/



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LiftingArms;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.DriveTrain.DriveTrainState;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.LiftingArms.LiftingArmsState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.LedController;
import frc.robot.Subsystems.Vision.Limelight;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;

  private Limelight limelight;
  //private LedController ledController;
  //private UsbCamera camera1;
  
  @Override
  public void robotInit() {
    limelight = Limelight.getInstance(DriveTrain.getInstance());
    //ledController = new LedController();
    //camera1 = CameraServer.startAutomaticCapture(0);
    robotContainer = new RobotContainer();
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);
    distOnboard.setAutomaticMode(true);

    double Voltaje = RobotController.getBatteryVoltage();
    DataLogManager.start();
    DataLogManager.log("Si estoy jalando " + RobotController.getFPGATime());
    addPeriodic(robotContainer.controlLoop(), 0.01, 0.005);
    DriverStation.reportWarning("Robot iniciao correctamente", false);
    DataLogManager.log("Voltaje actual de la bateria: " + Voltaje);
  } 
  
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
    DriveTrain.getInstance().setAngleToTarget(limelight.angloToTarget());
    Shooter.getInstance().setShooterAngle(limelight.getShooterAngle());
    /* 
    Esto sera implementado a futuro y seran errores al iniciar el robot. (Ejemplo: Saber si las conexiones de los Swerves estan bien hechas.)
    try {
      
    } catch(Exception e) {
      DataLogManager.log("Error detectado en: " + e.getMessage());
      DriverStation.reportError("Error detectado en: " + e.getMessage(), e.getStackTrace());
    }
    */
  } 

  @Override
  public void autonomousInit(){
    DriveTrain.getInstance().setState(DriveTrainState.AUTO).schedule();
    autonomousCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      DataLogManager.log("Automono Iniciado Correctamente.");
      autonomousCommand.schedule();
    } else {
      DataLogManager.log("Error al iniciar el automono. (Comando no encontrado.)");
    }
  }

  /*
  Va a ser util para el funcionamiento de reconocimiento de objetos. (Funciona igual que el periodico, simplemente se correra cuando este en modo autonomo.)
  @Override
  public void autonomousPeriodic() {} 
  */


  @Override
  public void teleopInit(){
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    //DataLogManager.log("Teleop Iniciado.");
    DriveTrain.getInstance().setState(DriveTrainState.JOYSTICKS).schedule();
    Intake.getInstance().setState(IntakeState.STOP).schedule();
    Shooter.getInstance().setState(ShooterState.STOP).schedule();
    LiftingArms.getInstance().setState(LiftingArmsState.STOP).schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit(){
    DriveTrain.getInstance().setState(DriveTrainState.IDLE).schedule();
    Intake.getInstance().setState(IntakeState.STOP).schedule();
    Shooter.getInstance().setState(ShooterState.STOP).schedule();
    LiftingArms.getInstance().setState(LiftingArmsState.STOP).schedule();
  }

  @Override
  public void disabledInit() {
    //distOnboard.setAutomaticMode(false);
    DataLogManager.log("Robot deshabilitado." + Timer.getFPGATimestamp());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}