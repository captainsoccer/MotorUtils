package util.MarinersController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class MarinersMeasurementsCTRE extends MarinersMeasurements {
    BaseStatusSignal[] signals;

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, double gearRatio) {
        super(
                positionSignal::getValueAsDouble,
                gearRatio
        );

        signals = new BaseStatusSignal[]{positionSignal};
    }

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, double gearRatio, double refreshHz) {
        super(
                positionSignal::getValueAsDouble,
                gearRatio
        );

        positionSignal.setUpdateFrequency(refreshHz);

        signals = new BaseStatusSignal[]{positionSignal};
    }

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, StatusSignal<AngularVelocity> velocitySignal,
                                    double gearRatio) {
        super(
                positionSignal::getValueAsDouble,
                velocitySignal::getValueAsDouble,
                gearRatio
        );

        signals = new BaseStatusSignal[]{positionSignal, velocitySignal};
    }

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, StatusSignal<AngularVelocity> velocitySignal,
                                    double gearRatio, double refreshHz) {
        super(
                positionSignal::getValueAsDouble,
                velocitySignal::getValueAsDouble,
                gearRatio
        );

        positionSignal.setUpdateFrequency(refreshHz);
        velocitySignal.setUpdateFrequency(refreshHz);

        signals = new BaseStatusSignal[]{positionSignal, velocitySignal};
    }

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, StatusSignal<AngularVelocity> velocitySignal,
                                    StatusSignal<AngularAcceleration> accelerationSignal, double gearRatio) {
        super(
                positionSignal::getValueAsDouble,
                velocitySignal::getValueAsDouble,
                accelerationSignal::getValueAsDouble,
                gearRatio
        );

        signals = new BaseStatusSignal[]{positionSignal, velocitySignal, accelerationSignal};
    }

    public MarinersMeasurementsCTRE(StatusSignal<Angle> positionSignal, StatusSignal<AngularVelocity> velocitySignal,
                                    StatusSignal<AngularAcceleration> accelerationSignal, double gearRatio, double refreshHz) {
        super(
                positionSignal::getValueAsDouble,
                velocitySignal::getValueAsDouble,
                accelerationSignal::getValueAsDouble,
                gearRatio
        );

        positionSignal.setUpdateFrequency(refreshHz);
        velocitySignal.setUpdateFrequency(refreshHz);
        accelerationSignal.setUpdateFrequency(refreshHz);

        signals = new BaseStatusSignal[]{positionSignal, velocitySignal, accelerationSignal};
    }


    @Override
    public void update(double dt) {
        BaseStatusSignal.refreshAll(signals);

        super.update(dt);
    }
}
