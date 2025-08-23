import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawDetection;

//object detection

//repositioning for intake
//movement for intake

public class AutomaticIntakeCommand extends Command {
    //creates a new AutomaticIntakeCommand
    public AutomaticIntakeCommand() {

    }

    public void initialize() {
        
    }

    public void execute() {
        RawDetection[] rawDetection = LimelightHelpers.getRawDetections("limelight-rear");
        if (rawDetection.length != 0) {
            SmartDashboard.putNumber("Field", rawDetection[0].classId);
        }
        


    }

    public void end() {

    }

    public boolean isFinished() {

    }


}