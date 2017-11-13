package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.utils.oldSensors;

import java.util.Iterator;


@TeleOp(name="Color Value Test", group="Utilities")
//@Disabled
public class colorValueTest extends OpMode {
    @Override
    public void init() {
        oldSensors.initialize(this,true);
    }


    @Override
    public void loop() {
        telemetry.addData("left red value" , oldSensors.leye.red());
        telemetry.addData("left blue value", oldSensors.leye.blue());

        telemetry.addData("right red value", oldSensors.reye.red());
        telemetry.addData("right blue value", oldSensors.reye.blue());

        Iterator<ColorSensor> iter = hardwareMap.colorSensor.iterator();
        while (iter.hasNext()) {
            ColorSensor sensor = iter.next();
            telemetry.addData("Color I2C Address", Integer.toHexString(sensor.getI2cAddress().get8Bit()));
        }
        telemetry.update();


    }
}
