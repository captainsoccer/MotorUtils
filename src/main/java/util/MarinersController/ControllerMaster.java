package util.MarinersController;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ControllerMaster extends SubsystemBase {

    private static ControllerMaster instance;

    private final ArrayList<MarinersController> controllers = new ArrayList<>();

    private final ArrayList<Notifier> notifiers = new ArrayList<>();

    public static final double ON_RIO_CONTROLLER_HZ = 50;

    public static final double MOTOR_CONTROLLER_HZ = 50;

    public static ControllerMaster getInstance(){
        if(instance == null){
            instance = new ControllerMaster();
        }
        return instance;
    }

    private ControllerMaster(){

    }

    public double addController(MarinersController controller, MarinersController.ControllerLocation location){
        controllers.add(controller);
        Notifier notifier = new Notifier(controller::runController);
        notifier.setName(controller.name + " Notifier");
        notifiers.add(notifier);
        double runningHZ = switch (location){
            case MOTOR -> MOTOR_CONTROLLER_HZ;
            case RIO -> ON_RIO_CONTROLLER_HZ;
        };

        notifier.startPeriodic(1 / runningHZ);

        return runningHZ;
    }

    public void stopLoop(){
        for(Notifier notifier : notifiers){
            notifier.stop();
        }
    }

    @Override
    public void periodic() {
        for(MarinersController controller : controllers){
            controller.update();
        }
    }
}
