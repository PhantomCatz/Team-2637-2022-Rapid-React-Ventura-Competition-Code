package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class CatzYdexer 
{    
    // Thread
    private       Thread yDexerThread;
    private final double YDEXER_THREAD_DELAY = 0.005;

    public Timer ydexertimer;

    //Ultrasonic sensor variables
    private final int USONIC_CARGO_IN_RANGE_DIO_PORT = 0; 
    public DigitalInput uSonicCargoInRangeInput;
    public boolean cargoInRange = false;

    //limit switch variables
    public SparkMaxLimitSwitch ydexerBtmLimitSwitch;  
    public SparkMaxLimitSwitch ydexerTopLimitSwitch;  
      
    private final boolean BALL_PRESENT     = true;
    private final boolean BALL_NOT_PRESENT = false;

    private boolean btmLimitSwitchState = BALL_NOT_PRESENT;
    private boolean topLimitSwitchState = BALL_NOT_PRESENT;

    // Motor and Motor Controller
    public CANSparkMax yDexerMtrCtrl;

    private final int INDEXER_MC_CAN_ID        = 40;
    private final int INDEXER_MC_CURRENT_LIMIT = 60;

    private final double YDEXER_SHOOT_MTR_PWR    = -0.55;
    private final double YDEXER_MOTOR_POWER_ON   = -0.55;
    private final double YDEXER_MOTOR_POWER_OFF  =  0.0;
    private final double YDEXER_SHOOT_SECOND_MTR_PWR = YDEXER_SHOOT_MTR_PWR + 0.13;

    private final int IGNORE_LIMIT_SWITCH_COUNT_TIME = ((int)(0.015 / YDEXER_THREAD_DELAY)) + 1;
    private final int YDEXER_SPEED_CHANGE_CNT = ((int)(0.02 / YDEXER_THREAD_DELAY)) + 1; ;

    private int yDexerCount = 0;

    private final boolean YDEXER_SHOOT_ALL = true;
    private final boolean YDEXER_SHOOT_ONE = false;

    
    

    // Ball Management
    public  boolean yDexerOn  = false;
    private boolean shooterOn = false;   
    private boolean turnShooterOff = false;
    private boolean checkStopConditions = true;
    private boolean yDexerShootMode = YDEXER_SHOOT_ALL;
    

    public CatzYdexer() 
    {

        //setup for motor controller
        yDexerMtrCtrl = new CANSparkMax(INDEXER_MC_CAN_ID, MotorType.kBrushless); 

        yDexerMtrCtrl.restoreFactoryDefaults();
        yDexerMtrCtrl.setIdleMode(IdleMode.kBrake);
        yDexerMtrCtrl.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);
        //indexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
        
        //setup for limit switch
        ydexerBtmLimitSwitch = yDexerMtrCtrl.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ydexerTopLimitSwitch = yDexerMtrCtrl.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        //no stop on detection
        ydexerBtmLimitSwitch.enableLimitSwitch(false); 
        ydexerTopLimitSwitch.enableLimitSwitch(false);

        //setup for ultrasonic
        //uSonicCargoInRangeInput = new DigitalInput(USONIC_CARGO_IN_RANGE_DIO_PORT);    
        
        ydexertimer = new Timer();
        startYDexerThread();
    }
 
    public void startYDexerThread()
    {
        yDexerThread = new Thread(() ->
        {     
            yDexerOn = false;
            yDexerCount = 0;
            checkStopConditions = true;
            
            while(true)
            {  
                if(shooterOn)
                {
                    
                    yDexerOn = true;
                    yDexerMtrCtrl.set(YDEXER_SHOOT_MTR_PWR);  //TBD MAKE CONSTANT

                    if (yDexerShootMode == YDEXER_SHOOT_ALL)
                    {
                        if(yDexerCount > YDEXER_SPEED_CHANGE_CNT)
                        {
                            yDexerMtrCtrl.set(YDEXER_SHOOT_SECOND_MTR_PWR);
                        }

                        if (turnShooterOff == true)
                        {
                            
                            yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                            yDexerOn       = false;
                            turnShooterOff = false;  
                            shooterOn      = false;
                        }    
                    }
                    else //Shoot 1 Cargo
                    {
                        /*--------------------------------------------------------------------------------
                        *  If there is:
                        *    2 cargo stored       : Turn off when top limit is true 
                        *    1 cargo stored at Top: Turn off after timeout
                        *    1 cargo stored at Btm: Ignore next top limit switch true & Turn off after timeout 
                        *-------------------------------------------------------------------------------*/
                    }

                    if (turnShooterOff == true)
                    {
                        yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                        yDexerOn       = false;
                        turnShooterOff = false;  
                    }
                }
                else
                {
                    /*------------------------------------------------------------------------------------
                    *  We are indexing cargo.  Grab sensor status
                    *
                    *  cargoNearYdexerEntrance  S-Btm  S-Top
                    *           0                 0      0      Do nothing
                    *           0                 1      0      Do nothing
                    *           0                 0      1      Do nothing
                    *           0                 1      1      Do nothing
                    *           1                 0      0      MOVE BALL - No cargo currently stored
                    *           1                 1      0      MOVE BALL - ONE cargo currently stored AND SPACE AVAILABLE AT TOP
                    *           1                 0      1      Do nothing - Cargo at top
                    *           1                 1      1      Do nothing - Cargo at top
                    *-----------------------------------------------------------------------------------*/
                    btmLimitSwitchState = ydexerBtmLimitSwitch.isPressed();
                    topLimitSwitchState = ydexerTopLimitSwitch.isPressed();

                    cargoInRange = Robot.lidar.isInRange(5.5,3.0);  //TBD - WE CAN READ HERE FOR DEBUG OR WE CAN ONLY READ WHEN NEEDED
                    //System.out.println(cargoInRange);

                    if(yDexerOn == true)
                    {
                        /*--------------------------------------------------------------------------------
                        *  YDexer is ON, so check conditions to see if it is time to turn YDexer OFF. 
                        *  Since Cargo can still be on a limit switch on next iteration after turning
                        *  YDexer ON, we need to wait for Cargo to move off of limit switch before
                        *  checking to turn YDexer OFF.
                        *-------------------------------------------------------------------------------*/
                        //System.out.println("C" + btmLimitSwitchState + topLimitSwitchState);
                                                
                        if(checkStopConditions == false)
                        {
                            if(yDexerCount > IGNORE_LIMIT_SWITCH_COUNT_TIME)
                            {
                                checkStopConditions = true;
                            }
                            else
                            {
                                yDexerCount++;
                            }                            
                        }

                        if(checkStopConditions == true)
                        {   
                            if(btmLimitSwitchState == BALL_PRESENT || topLimitSwitchState == BALL_PRESENT)
                            {
                                yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                                yDexerOn       = false;
                            }
                        }
                    }
                    else
                    {
                        /*--------------------------------------------------------------------------------
                        *  YDexer is OFF, so check if Cargo is in range.  If it is in range, Turn YDexer ON
                        *-------------------------------------------------------------------------------*/
                        if(cargoInRange == true)
                        {
                            cargoInRange = false;   //TBD THIS DOESN'T MATTER SINCE WE WILL UPDATE ON NEXT ITERATION

                            if(topLimitSwitchState == BALL_NOT_PRESENT)
                            {
                                checkStopConditions = false;
                                yDexerOn            = true;
                                yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_ON);
                                yDexerCount = 0;
                            }
                        } 
                    }
                }
            
                Timer.delay(YDEXER_THREAD_DELAY); 
            }
        }); //end of thread
        yDexerThread.start();
    }
    
    
    /*--------------------------------------------------------------------------------
    *
    *  Methods to notify Ydexer of Shooter state 
    *
    *-------------------------------------------------------------------------------*/
    public void setShooterOn()
    {
        shooterOn      = true;
        turnShooterOff = false;
        yDexerCount = 0;
    }
    
    public void setShooterOff()
    {
        turnShooterOff = true;
       
    }


    /*--------------------------------------------------------------------------------
    *
    *  Manual Y Dexer Control
    *
    *-------------------------------------------------------------------------------*/
    public void ydexerMotorPowerOn()
    {
        yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_ON);
    }

    public void ydexerMotorPowerOff()
    {
        yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
    }

    public void setYdexerSpeed(double speed)
    {
        yDexerMtrCtrl.set(speed);
    }


    /*--------------------------------------------------------------------------------
    *
    *  SmartDashboard
    *
    *-------------------------------------------------------------------------------*/
    public void smartDashboardYdexer()
    {
        SmartDashboard.putBoolean("Cargo In Range", cargoInRange);
        SmartDashboard.putBoolean("BotLimitSwitch", btmLimitSwitchState);
        SmartDashboard.putBoolean("TopLimitSwitch", topLimitSwitchState);
    }


    public void smartDashboardYdexer_DEBUG()
    {
        SmartDashboard.putBoolean("ShooterOn", shooterOn);
        SmartDashboard.putBoolean("Ydexer",    yDexerOn);
    }

}