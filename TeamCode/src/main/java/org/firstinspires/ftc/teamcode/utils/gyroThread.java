package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class gyroThread extends Thread{
    private static ElapsedTime gyroTime = new ElapsedTime();
    public void run()
    {
        while (true)
        {
            if(gyroTime.milliseconds() >= 20)
            {
                gyroTime.reset();
                Sensors.angles = Sensors.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            }
        }
    }
}
