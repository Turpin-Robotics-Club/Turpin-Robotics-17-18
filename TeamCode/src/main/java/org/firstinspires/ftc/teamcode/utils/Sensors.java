package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerVelocityParams;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.utils.oldSensors.gyro;


public class Sensors {

    public static double gyrochange;
    private static double timeAutonomous;
    private static ElapsedTime runtime = new ElapsedTime();
    public static int gyroInitial;
    //public static ColorSensor leye;
    //public static ColorSensor reye;
    public static BNO055IMU gyro;
    static boolean red;
    static  double trueHeading;
    public static double driverOffset = 0;

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
        //leye = hardware_map.get(ColorSensor.class, "leye");
        //leye.setI2cAddress(I2cAddr.create8bit(0x4c));
        //leye.enableLed(false);
        red = reds;

        telemetry = opMode.telemetry;

        BNO055IMU.Parameters IMUparams = new BNO055IMU.Parameters();
        //reye = hardware_map.get(ColorSensor.class, "reye");
        //reye.setI2cAddress(I2cAddr.create8bit(0x5c));
        //reye.enableLed(false);

        gyro = hardware_map.get(BNO055IMU.class, "imu");

        driverOffset = 0;

        runtime.reset();
        /*
        while (gyro.isCalibrating() && opMode.opModeIsActive()) {
            //telemetry.addData("Time", runtime.seconds());
            //telemetry.update();
            for (int i = 0; i < 10000; i++);
        }
        //telemetry.addData("Done", "");
        //telemetry.update();
        gyroInitial = gyro.getHeading();
        */
        gyro.initialize(IMUparams);


    }


    public static void gyroDriftRead() {

        if(gyro.getHeading() == 0) {
            gyrochange= 0;
        } else if (gyro.getHeading() < (gyroInitial + 180) - 360) {
            gyrochange = -((360 - gyroInitial) + gyro.getHeading()) / runtime.seconds();
        } else if (gyro.getHeading() > gyroInitial + 180) {
            gyrochange = (gyroInitial + (360 - gyro.getHeading())) / runtime.seconds();
        } else {
            gyrochange = (gyroInitial - gyro.getHeading()) / runtime.seconds();
        }
    }

    public static double gyroIntegratedHeading() {
        return (gyrochange * (runtime.seconds())) + gyro.getIntegratedZValue();
    }

    public static double gyroHeading()
    {
        //calculate modified heading
        trueHeading = (gyrochange * runtime.seconds()) + gyro.getHeading();

        //make sure it's in the gyro's range
        while (0 > trueHeading || trueHeading >= 360) {
            if (trueHeading >= 360) {
                trueHeading = trueHeading - 360;
            }
            if (trueHeading < 0) {
                trueHeading = trueHeading + 360;
            }
        }
        //if it is, then return
        return trueHeading;
    }

    public static void resetGyro()
    {
        gyro.resetZAxisIntegrator();
        runtime.reset();
    }

    public static void offsetReset() {
        telemetry.addData(">", "gyro resetting");
        telemetry.update();
        gyro.calibrate();
        gyrochange = 0;
    }


}
