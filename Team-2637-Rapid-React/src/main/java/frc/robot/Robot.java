package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.DataLogger.*;
import frc.Autonomous.CatzAutonomous;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzClimber;
import frc.Mechanisms.CatzDriveTrain;
import frc.Mechanisms.CatzShooter;
//import frc.Mechanisms.CatzShydexer;
import frc.Mechanisms.CatzTurret;
import frc.Mechanisms.CatzVision;
import frc.Mechanisms.CatzYdexer;
import frc.Mechanisms.LiDAR;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static CatzDriveTrain driveTrain;
  public static CatzAutonomous auton;
  public static CatzIntake     intake;
  public static CatzShooter    shooter;
  public static CatzTurret     turret;
  public static CatzVision     vision;
  public static CatzYdexer     ydexer;
  //public static CatzShydexer   shydexer;
  public static DataCollection dataCollection;
  public static LiDAR          lidar;
  public static CatzClimber    climb;
  public static CatzLog        catzLog;
  public static CatzConstants constants;

  public static AHRS navx;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;
  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

  public static PowerDistribution pdp;
  public static Timer dataCollectionTimer;


  public ArrayList<CatzLog> dataArrayList;

  private UsbCamera driverCamera;

  private static int cameraResolutionWidth = 240;
  private static int cameraResolutionHeight = 160;
  private static int cameraFPS = 30;

  public double drvTrainPwrFactor = 1.0;
  

  double left;
  double right;

  boolean firstTele = true;
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    pdp = new PowerDistribution();
    navx = new AHRS(Port.kMXP, (byte) 200);
    navx.reset();

    dataCollection = new DataCollection();
    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    auton      = new CatzAutonomous();
    shooter    = new CatzShooter();
    vision     = new CatzVision();
    turret     = new CatzTurret();
    climb      = new CatzClimber();
    lidar      = new LiDAR();
    //shydexer   = new CatzShydexer();
    ydexer     = new CatzYdexer();
    constants  = new CatzConstants();
    
    
    dataArrayList = new ArrayList<CatzLog>();
    
    dataCollectionTimer = new Timer();

    dataCollection.dataCollectionInit(dataArrayList);

    auton.initializePathOptions();
    
    driverCamera = CameraServer.startAutomaticCapture(0);
    driverCamera.setFPS(cameraFPS);
    driverCamera.setResolution(cameraResolutionWidth, cameraResolutionHeight);
    driverCamera.setPixelFormat(PixelFormat.kMJPEG);

    driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    

  } // End of robotInit()

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // creates the pathway choice in shuffleboard RIC 2/19/2022
    //SmartDashboard.putBoolean(CatzConstants.ALLIANCE_COLOR, true);
    //SmartDashboard.put(CatzConstants.ALLIANCE_POSITION, true);


    /*SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR2, true);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR3, true);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR4, true);

    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR1, false);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR2, false);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR3, false);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTOR4, false);*/
    

    //Robot.auton.smartDashBoardAutonomous_DEBUG();
    auton.smartDashBoardAutonomous();
    lidar.smartDashboardLiDAR();
    shooter.smartDashboardShooter();
    turret.smartDashboardTurret();
    ydexer.smartDashboardYdexer();
    //auton.determinePath();

    
  
  }



  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    shooter.inAutonomous = true;
    driveTrain.setToBrakeMode();
    
    
    int tempPathChoice = 0;
    /*if (SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTOR1, false)) 
    {
      tempPathChoice = 1;
    }
    else if (SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTOR2, false)) 
    {
      tempPathChoice = 2;
    } 
    else if (SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTOR3, false)) 
    {
      tempPathChoice = 3;
    } 
    else if (SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTOR4, false)) 
    {
      tempPathChoice = 4;
    }*/

    //check position

    tempPathChoice = 1;

    switch (tempPathChoice) 
    {
      case (1):
        auton.pathFenderShoot();
        
        //auton.pathShootOnly();
        break;

      case (2):
        auton.pathTwo();
        break;

      case (3):
        auton.pathOne();
        //auton.pathOneLeft();
        break;

      case (4):
        auton.testPath();
        break;
    }
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    
  }



  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    shooter.inAutonomous = false;
    
    
    
    if(firstTele == true)
    {
      driveTrain.instantiateDifferentialDrive();
      firstTele = false;
    }
    driveTrain.setToBrakeMode();

    /*dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_SHOOTER);
    dataCollection.startDataCollection();*/
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    //intake.intakeRollerIn();

    //---------------vision----------------// 
    vision.turretTracking();
    
    //--------------drivetrain------------//
    if(xboxDrv.getAButtonPressed())
    {
      drvTrainPwrFactor= 0.85;
    }
    else if(xboxDrv.getBButtonPressed())
    {
      drvTrainPwrFactor = 1.0;
    }


    if(xboxAux.getRightBumper())
    {
      driveTrain.arcadeDrive((xboxAux.getLeftY()), xboxAux.getRightX() );
    }
    else
    {
      left = xboxDrv.getLeftY();
      right = xboxDrv.getRightX();
      System.out.println("L :" + left + " R " + right);
      
      //driveTrain.arcadeDrive(left * drvTrainPwrFactor, xboxDrv.getRightX() * drvTrainPwrFactor);
      driveTrain.arcadeDrive((left * drvTrainPwrFactor) , (right * drvTrainPwrFactor));
      
    } 

    //driveTrain.arcadeDrive(0.8, 0.0);

    if (xboxDrv.getLeftBumper()) 
    {
      driveTrain.shiftToHighGear();
    } 
    else if (xboxDrv.getRightBumper()) 
    {
      driveTrain.shiftToLowGear();
    }

    

    //-------------------------intake + shydexer--------------------------//
    
    if (xboxDrv.getLeftTriggerAxis() > 0.2)
    {
      intake.intakeRollerOut();
      //shydexer.shyDexerReverse(); sfdsdd
    }
    else if (xboxDrv.getRightTriggerAxis() > 0.2)
    {
      intake.intakeRollerIn();
      //shydexer.shyDexerOn();
    }
    else
    {
      intake.intakeRollerOff();
      //shydexer.shyDexerOff();
    }
    

    if (xboxDrv.getLeftStickButton())
    {
      intake.deployIntake(); 
    }
    else if (xboxDrv.getRightStickButton())
    {
      intake.stowIntake();
    }

    //--------------------------turret--------------------//
    if ((xboxAux.getRightX() >=  0.2 && xboxAux.getRightBumper() == false) ||
        (xboxAux.getRightX() <= -0.2 && xboxAux.getRightBumper() == false)) 
    {
      turret.turretState = turret.TURRET_STATE_MANUAL;
      if (xboxAux.getXButton() == true) 
      {
        turret.rotateTurret(xboxAux.getRightX() * turret.TURRET_LOW_RATE);
      } 
      else 
      {
        turret.rotateTurret(xboxAux.getRightX() * turret.TURRET_HIGH_RATE);
      }
    } 
    else if (turret.turretState != turret.TURRET_STATE_MOVE_TO_POSITION) 
    {
      turret.stopTurret();
    }


    if (xboxAux.getAButtonPressed()) //TBD remap
    {
      //turret.moveToHome();
    }

    if (xboxAux.getYButton()) 
    {
      turret.rotateTurret(vision.getXOffset() * 0.015);
    }

    //-------------------shooter--------------------//
 
    if (xboxAux.getBButtonPressed()) 
    {
      shooter.shoot();
    }
    else if (xboxAux.getStartButton()) 
    {
      shooter.shooterOff();
    } 

    if (xboxAux.getPOV() == DPAD_DN) 
    {                             
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_FENDER); //TBD review after ventura
    } 
    else if (xboxAux.getPOV() == DPAD_UP) 
    {                             
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_FAR);
    } 

    if (xboxAux.getPOV() == DPAD_RT) 
    {
      shooter.hoodSetTop();
    }
    else if (xboxAux.getPOV() == DPAD_LT)
    { 
      shooter.hoodSetBot(); 
    }
    
    //----------------------ydexer--------------------//

    //manual ydexer control
    if(xboxDrv.getYButtonPressed())
    {
      //shydexer.shyDexerOn();
      lidar.manualOveride = true;
      //ydexer.cargoInRange = true; //go up
    }  
    
    
    //----------------------------Climb---------------------------------//
    if(xboxAux.getRightBumper() && Math.abs(xboxAux.getRightTriggerAxis()) > 0.1)
    {
      climb.climbElevatorManualCntrl(xboxAux.getRightTriggerAxis()); //down
    }
    else if(xboxAux.getRightBumper() && Math.abs(xboxAux.getLeftTriggerAxis()) > 0.1)
    {
      climb.climbElevatorManualCntrl(-xboxAux.getLeftTriggerAxis()); //up
    }
    else //if(xboxAux.getAButton() == false)
    {
      climb.climbElevatorManualCntrl(0.0);
    }

    if(xboxAux.getRightBumper() && Math.abs(xboxDrv.getLeftY()) > 0.1)
    {
      if(xboxDrv.getXButton() == true)
      {
        climb.climbExtendRightRotatingArmManualCtrl(-xboxDrv.getLeftY());
      }
      else if(xboxDrv.getBButton() == true)
      {
        climb.climbExtendLeftRotatingArmManualCtrl(-xboxDrv.getLeftY());
      }
      else 
      {
        climb.climbExtendRotatingArmManualCtrl(-xboxDrv.getLeftY());
      }
    }
    else //if(xboxAux.getAButton() == false)
    {
      climb.climbExtendRotatingArmManualCtrl(0.0);
    }

    if(xboxAux.getRightBumper() && Math.abs(xboxDrv.getRightY()) > 0.1)
    {
      climb.climbRotatingArmManualCntrl(xboxDrv.getRightY());
    }
    else //if(xboxAux.getAButton() == false)
    {
      climb.climbRotatingArmManualCntrl(0.0);
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    dataCollection.stopDataCollection();
    driveTrain.setToCoastMode();

    try 
    {
      dataCollection.exportData(dataArrayList);
    } 
    catch (Exception e) 
    {
      e.printStackTrace();
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
