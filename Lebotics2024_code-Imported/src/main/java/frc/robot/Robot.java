package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
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

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  //private UsbCamera camera1;
  
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    //camera1 = CameraServer.startAutomaticCapture(0);
    addPeriodic(robotContainer.controlLoop(), 0.01, 0.005);
  } 
  
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  } 

  @Override
  public void autonomousInit(){
    DriveTrain.getInstance().setState(DriveTrainState.AUTO).schedule();
    autonomousCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit(){
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  
}

