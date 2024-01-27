package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LiftingArms;
import frc.robot.Subsystems.Intake.IntakeState;

public class Robot extends TimedRobot {
  
  private final XboxController controller = new XboxController(0);
  private final DriveTrain swerve = DriveTrain.getInstance();
  private final LiftingArms liftingArms = LiftingArms.getInstance();
  private final Intake intake = Intake.getInstance();
  private double DEADBAND = 0;
  private boolean FieldRelativeTeleop = true;

  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("FieldRelativeTeleop", FieldRelativeTeleop);
    SmartDashboard.putNumber("DeadBand_Drift", DEADBAND);
  } 
  
  @Override
  public void robotPeriodic(){
    swerve.periodic();
    liftingArms.periodic();
    intake.periodic();
  } 

  @Override
  public void teleopPeriodic() {
    boolean FRT = SmartDashboard.getBoolean("FieldRelativeTeleop", true);
    if(FRT != FieldRelativeTeleop){ FieldRelativeTeleop = FRT;}
    double DZ = SmartDashboard.getNumber("DeadBand_Drift", 0);
    if(DZ != DEADBAND){ DEADBAND = DZ;}
    driveWithController(FieldRelativeTeleop);
    liftWithController();
    intakeWithController();
  }

  @Override
  public void teleopExit(){
    liftingArms.stopArms();
  }

  private void driveWithController(boolean fieldRelative){
    double xSpeed = MathUtil.applyDeadband(controller.getLeftX(), DEADBAND);
    double ySpeed = -MathUtil.applyDeadband(controller.getLeftY(), DEADBAND);
    double rot = -MathUtil.applyDeadband(controller.getRightX(), DEADBAND);
    swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private void liftWithController(){
    int pov = controller.getPOV();
    if(pov == 0){
      liftingArms.extendArms();
    }else if(pov == 180){
      liftingArms.retractArms();
    }else{
      liftingArms.stopArms();
    }
  }

  private void intakeWithController(){
    if(controller.getBButton()){
      intake.setState(IntakeState.STOP);
    }else if(controller.getAButton()){
      intake.setState(IntakeState.DOWN);
    }else if(controller.getXButton()){
      intake.setState(IntakeState.UP);
    }else if(controller.getYButton()){
      intake.setState(IntakeState.CUSTOM);
    }else if(controller.getRightBumper()){
      intake.shoot();
    }
    intake.runState();
    
  }
}

