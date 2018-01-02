package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;


public class Sensors {

    public static double gyrochange;

    private static ElapsedTime runtime = new ElapsedTime();
    public static double gyroInitial = 0;
    public static BNO055IMU gyro;
    public static boolean red;
    public static Orientation angles;
    private static LinearOpMode opMode;
    private static Telemetry telemetry;


    /**
     *
     * @param _opMode to get FTC data
     * @param reds whether or not we are on the red team
     */
    public static void initialize(OpMode _opMode, boolean reds) {
        if (_opMode instanceof LinearOpMode) {
            opMode = (LinearOpMode) _opMode;
        } else {
            return;
        }
        HardwareMap hardware_map = opMode.hardwareMap;
        red = reds;

        telemetry = opMode.telemetry;


        BNO055IMU.Parameters IMUparams = new BNO055IMU.Parameters();
        IMUparams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparams.calibrationDataFile = "Calibfile.json";
        IMUparams.loggingEnabled      = true;
        IMUparams.loggingTag          = "IMU";
        IMUparams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardware_map.get(BNO055IMU.class, "imu");



        runtime.reset();

        gyro.initialize(IMUparams);
        sleep(10000);
        move.pause(100);
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("angle", angles);


        //+180 for the orientation of the rev module
        if(realGyro()+180>360)
            gyroInitial = realGyro() +180 -360;
        else
            gyroInitial = realGyro() +180;


        ((LinearOpMode) _opMode).waitForStart();
        gyroDriftRead();
    }






    private static void gyroDriftRead() {

        if (realGyro() == gyroInitial)
            gyrochange=0;
        else if (realGyro()<gyroInitial && realGyro()>gyroInitial-180)
            gyrochange = (realGyro() - gyroInitial) / runtime.seconds();
        else if (realGyro()>gyroInitial+180 && realGyro()<gyroInitial+360)
            gyrochange = -(gyroInitial + (360-realGyro())) / runtime.seconds();
        else if (realGyro()>gyroInitial && realGyro()<gyroInitial+180)
            gyrochange = (realGyro() - gyroInitial) / runtime.seconds();
        else if (realGyro()<gyroInitial+180 && realGyro()>gyroInitial-360)
            gyrochange = (gyroInitial + (360-realGyro())) / runtime.seconds();
    }

    public static double readGyro() {
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if ((gyrochange * (runtime.seconds())) + realGyro()-gyroInitial<0)
            return (gyrochange * (runtime.seconds())) + realGyro()  -gyroInitial  +360;
        else if ((gyrochange * (runtime.seconds())) + realGyro()-gyroInitial>360)
            return (gyrochange * (runtime.seconds())) + realGyro()  -gyroInitial  -360;
        else
            return (gyrochange * (runtime.seconds())) + realGyro()-gyroInitial;
    }

    public static void resetGyro()
    {
        gyroInitial=realGyro();
        runtime.reset();
        gyrochange = 0;
    }

    private static double realGyro()
    {

        return 180-angles.firstAngle;

    }

/*    public static class gyroThread extends Thread{
        private static ElapsedTime gyroTime = new ElapsedTime();
        boolean end = false;
        public void run()
        {
            while (!end)
            {
                if(gyroTime.milliseconds() >= 20)
                {
                    gyroTime.reset();
                    angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                }
            }
        }

        public void end( )
        {
            end=true;
        }

    }
*/
}
