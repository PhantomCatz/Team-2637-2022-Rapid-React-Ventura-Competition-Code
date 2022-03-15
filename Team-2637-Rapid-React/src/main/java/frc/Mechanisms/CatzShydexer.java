package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CatzShydexer
{
    private final double SHYDEXER_MOTOR_POWER_ON    =  1.0; 
    private final double SHYDEXER_MOTOR_POWER_OFF   =  0.0;
    private final double SHYDEXER_MOTOR_POWER_REV   = -0.5;
     
    //defining and adding the motors 
    private CANSparkMax shydexerMtrCtrl;
    
    private final int SHYDEXER_MC_CAN_ID        = 43; 
    private final int SHYDEXER_MC_CURRENT_LIMIT = 60;

    public CatzShydexer()
    {
     
     shydexerMtrCtrl = new CANSparkMax(SHYDEXER_MC_CAN_ID, MotorType.kBrushless);

     shydexerMtrCtrl.restoreFactoryDefaults();
     shydexerMtrCtrl.setIdleMode(IdleMode.kCoast);
     shydexerMtrCtrl.setSmartCurrentLimit(SHYDEXER_MC_CURRENT_LIMIT);
     shydexerMtrCtrl.set(SHYDEXER_MOTOR_POWER_OFF);
    }


    public void shyDexerOn()
    {
        shydexerMtrCtrl.set(SHYDEXER_MOTOR_POWER_ON);
    }
     
    public void shyDexerOff()
    {
        shydexerMtrCtrl.set(SHYDEXER_MOTOR_POWER_OFF);
    }

    public void shyDexerReverse()
    {
        shydexerMtrCtrl.set(SHYDEXER_MOTOR_POWER_REV);
    }

    public void shyDexerSet(double speed)
    {
        shydexerMtrCtrl.set(speed);
    }
    
}


