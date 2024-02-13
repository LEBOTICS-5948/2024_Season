package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
  
  private RobotContainer robotContainer;
  

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    addPeriodic(robotContainer.controlLoop(), 0.01, 0.005);
  } 
  
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  } 


  @Override
  public void teleopInit(){
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

  }

  
}

