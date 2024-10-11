package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;

// Recomiendo mantener estas funciones declaradas para disminuir la memoria que se destina en el robot.
/* 
Librerias que se puedan utilizar a futuro.
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.DriverStationJNI;
import javax.xml.crypto.Data;
import edu.wpi.first.util.datalog.DataLogReader;

Despues
import frc.robot.Subsystems.Functions.DistanceSensor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

Librerias siguientes a implementar.
import edu.wpi.first.wpilibj.PowerDistribution; // Saber el estado de la bateria y comunicarle al driver si es necesario cambiarla.
*/

import com.revrobotics.Rev2mDistanceSensor.Port;

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

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private LedController ledController;

  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;
  //private UsbCamera camera1;
  
  @Override
  public void robotInit() {
    //camera1 = CameraServer.startAutomaticCapture(0);
    ledController = new LedController(); // Nueva instancia para controlar las leds.
    robotContainer = new RobotContainer();
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);

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
    ledController.LedsFuncionar();
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
    } else {
      //DataLogManager.log("");
    }
    //DataLogManager.log("Teleop Iniciado.");
    distOnboard.setAutomaticMode(true);
    DriveTrain.getInstance().setState(DriveTrainState.JOYSTICKS).schedule();
    Intake.getInstance().setState(IntakeState.STOP).schedule();
    Shooter.getInstance().setState(ShooterState.STOP).schedule();
    LiftingArms.getInstance().setState(LiftingArmsState.STOP).schedule();
  }

  @Override
  public void teleopPeriodic() {
    if(distOnboard.isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", distOnboard.getRange()); // Intake Sensor
      SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp()); // Intake Sensor
    } 

    if(distMXP.isRangeValid()) {
      SmartDashboard.putNumber("Range MXP", distMXP.getRange()); // Shooter Sensor
      SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp()); // Shooter Sensor
    }
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
    distOnboard.setAutomaticMode(false);
    DataLogManager.log("Robot deshabilitado." + Timer.getFPGATimestamp());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    //System.out.println("Distancia medida: " + distance + " metros");
  }
}