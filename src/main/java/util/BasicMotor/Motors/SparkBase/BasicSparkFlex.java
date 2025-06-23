package util.BasicMotor.Motors.SparkBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.MotorManager;

/**
 * This class represents a basic Spark Flex motor controller.
 * it extends the BasicSparkBase class and provides specific implementations
 */
public class BasicSparkFlex extends BasicSparkBase {
    /**
     * creates a basic spark flex motor controller with the given gains and id
     *
     * @param gains     the gains of the motor controller
     * @param id        the id of the motor controller
     * @param name      the name of the motor controller
     * @param gearRatio the gear ratio of the motor controller
     * @param location  the location of the motor controller (RIO or MOTOR)
     */
    public BasicSparkFlex(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        super(new SparkFlex(id, SparkLowLevel.MotorType.kBrushless), new SparkFlexConfig(), gains, name, gearRatio, location);
    }

    /**
     * creates a basic spark flex motor controller with the given configuration
     *
     * @param config the configuration of the motor controller
     */
    public BasicSparkFlex(BasicMotorConfig config) {
        super(new SparkFlex(config.motorConfig.id, SparkLowLevel.MotorType.kBrushless), new SparkFlexConfig(), config);
    }

    @Override
    protected void configExternalEncoder(boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
        assert config instanceof SparkFlexConfig;

        var config = (SparkFlexConfig) this.config;

        //sets whether the absolute encoder is inverted or not
        config.externalEncoder.inverted(inverted);
        //sets the conversion factor for the absolute encoder position and velocity
        config.externalEncoder.positionConversionFactor(1 / sensorToMotorRatio);
        config.externalEncoder.velocityConversionFactor(1 / sensorToMotorRatio);
    }

    @Override
    protected RelativeEncoder getExternalEncoder() {
        assert motor instanceof SparkFlex;

        var sparkMax = (SparkFlex) motor;

        return sparkMax.getExternalEncoder();
    }
}
