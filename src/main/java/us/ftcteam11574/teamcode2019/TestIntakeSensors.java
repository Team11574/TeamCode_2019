package us.ftcteam11574.teamcode2019;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestIntakeSensors extends OpMode {
    Rev2mDistanceSensor mDistanceSensor;
    ColorSensor mColorSensor;

    @Override
    public void init() {
        mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        mColorSensor = hardwareMap.colorSensor.get("color");
    }

    @Override
    public void loop() {
        telemetry.addData("color",
                String.format("r:%d, g:%d, b:%d, a:%d",
                        mColorSensor.red(),
                        mColorSensor.green(),
                        mColorSensor.blue(),
                        mColorSensor.alpha()));
        telemetry.addData("distance (mm)", mDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
