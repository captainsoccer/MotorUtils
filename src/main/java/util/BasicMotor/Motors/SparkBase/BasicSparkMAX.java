package util.BasicMotor.Motors.SparkBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import util.BasicMotor.Gains.*;
import util.BasicMotor.MotorManager;

public class BasicSparkMAX extends BasicSparkBase {
    public BasicSparkMAX(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        super(new SparkMax(id, SparkLowLevel.MotorType.kBrushless), new SparkMaxConfig(), gains, name, gearRatio, location);
    }

    @Override
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio,
                                   double mechanismToSensorRatio, AbsoluteEncoderRange absoluteEncoderRange) {
        //sets the absolute encoder configuration
        config.absoluteEncoder.setSparkMaxDataPortConfig();

        super.useAbsoluteEncoder(inverted, zeroOffset, sensorToMotorRatio, mechanismToSensorRatio, absoluteEncoderRange);
    }

    @Override
    public void useExternalEncoder(boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
        config.absoluteEncoder.setSparkMaxDataPortConfig();

        super.useExternalEncoder(inverted, sensorToMotorRatio, mechanismToSensorRatio);
    }

    @Override
    protected void configExternalEncoder(boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
        assert config instanceof SparkMaxConfig;

        var config = (SparkMaxConfig) this.config;

        //sets the absolute encoder configuration
        config.alternateEncoder.setSparkMaxDataPortConfig();
        //sets whether the absolute encoder is inverted or not
        config.alternateEncoder.inverted(inverted);
        //sets the conversion factor for the absolute encoder position and velocity
        config.alternateEncoder.positionConversionFactor(1 / sensorToMotorRatio);
        config.alternateEncoder.velocityConversionFactor(1 / sensorToMotorRatio);
    }

    @Override
    protected RelativeEncoder getExternalEncoder() {
        assert motor instanceof SparkMax;

        var sparkMax = (SparkMax) motor;

        return sparkMax.getAlternateEncoder();
    }
}
