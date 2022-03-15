package frc.Mechanisms;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class CatzVision 
{
    public boolean hasValidTarget;
    public boolean inShootingRange;

    public double xOffset;
    public double yOffset;
    public double targetPresent;

    public double calcAngle;

    public final double HOOD_BTM_Y_POS = 0.0; //TBD
    public final double HOOD_TOP_Y_POS = 0.0; //TBD
    public final double HOOD_Y_OFFSET  = 1.0; //TBD

    public final double HOOD_TOP_MIN_POS = HOOD_TOP_Y_POS - HOOD_Y_OFFSET;
    public final double HOOD_TOP_MAX_POS = HOOD_TOP_Y_POS + HOOD_Y_OFFSET;
    public final double HOOD_BTM_MIN_POS = HOOD_BTM_Y_POS - HOOD_Y_OFFSET;
    public final double HOOD_BTM_MAX_POS = HOOD_BTM_Y_POS + HOOD_Y_OFFSET;



    public void turretTracking()
    {
        targetPresent = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        xOffset       = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        yOffset       = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        if (targetPresent == 1.0)
        {
            hasValidTarget = true;

            if (Robot.shooter.getHoodPosition() == Robot.shooter.HOOD_TOP_POS)
            {
                if (yOffset >= HOOD_TOP_MIN_POS && yOffset <= HOOD_TOP_MAX_POS)
                {
                    inShootingRange = true;
                }
                else
                {
                    inShootingRange = false;
                }
            }
            else
            {
                if (yOffset >= HOOD_BTM_MIN_POS && yOffset <= HOOD_BTM_MAX_POS)
                {
                    inShootingRange = true;
                }
                else
                {
                    inShootingRange = false;
                }
            }
        }
        else
        {
            hasValidTarget  = false;
            inShootingRange = false;
        }
        
        smartDashboardVision();
    }

    public double getXOffset()
    {
        return xOffset;
    }

    public void smartDashboardVision()
    {
        SmartDashboard.putNumber("Target Valid Num", targetPresent);
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);
        SmartDashboard.putNumber("X Offset", xOffset);
        SmartDashboard.putNumber("Y Offset", yOffset);
    }
}
