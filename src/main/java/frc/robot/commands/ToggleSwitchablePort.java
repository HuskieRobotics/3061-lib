package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.PowerDistribution;

public class ToggleSwitchablePort extends CommandBase{

    private final PowerDistribution powerDistribution;

    public ToggleSwitchablePort(PowerDistribution powerDistribution){
        this.powerDistribution = powerDistribution;
    }

    @Override
    public void initialize(){
        powerDistribution.setSwitchableChannel(!powerDistribution.getSwitchableChannel());
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}