// /*
// Copyright 2019 FIRST Tech Challenge Team 12886

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// */
// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.util.Range;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import java.text.DecimalFormat;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.robot.Robot;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import java.text.SimpleDateFormat;
// import java.util.Date;

// /**
//  * Mecanum teleop (with an optional arcade mode)
//  * * Left stick controls x/y translation.
//  * * Right stick controls rotation about the z axis
//  * * When arcade mode is enabled (press "a"), translation direction
//  * becomes relative to the field as opposed to the robot. You can
//  * reset the forward heading by pressing "x".
//  */
// @TeleOp(name = "Mecanum")
// public class MecanumCartesian extends OpMode {
//     public DecimalFormat format = new DecimalFormat("##.00");
//     private BNO055IMU imu;
//     private DcMotor frontLeft;
//     private DcMotor rearLeft;
//     private DcMotor frontRight;
//     private DcMotor rearRight;

//     public double leftStickY;
//     public double leftStickX;
//     public double rightStickX;

//     // public double FL_power_raw;
//     // public double FR_power_raw;
//     // public double RL_power_raw;
//     // public double RR_power_raw;
//     // public double FL_power;
//     // public double FR_power;
//     // public double RL_power;
//     // public double RR_power;

//     public float gyro_degrees;

//     public double newForward;
//     public double newStrafe;
//     public double newRotation;

//     // State used for updating telemetry
//     public Orientation angles;
//     public Acceleration gravity;

//     private boolean arcadeMode = false;
//     private int gyroCalibratedCount = 0;

//     @Override
//     public void init() {
//         motorInit();
//         gyroInit();
//         gamepadInit();

//         // robot = new Robot(hardwareMap, telemetry);
//         // robot.runUsingEncoders();
//         // controller = new Controller(gamepad1);
//     }

//     private void gamepadInit() {
//         gamepad1.setJoystickDeadzone(0.2f);
//     }

//     private void motorInit() {
//         frontLeft = hardwareMap.dcMotor.get("frontLeft");
//         rearLeft = hardwareMap.dcMotor.get("rearLeft");
//         frontRight = hardwareMap.dcMotor.get("frontRight");
//         rearRight = hardwareMap.dcMotor.get("rearRight");

//         frontLeft.setDirection(DcMotor.Direction.REVERSE);
//         rearRight.setDirection(DcMotor.Direction.REVERSE);
//         // rearRight.setDirection(DcMotor.Direction.REVERSE);
//         // frontRight.setDirection(DcMotor.Direction.REVERSE);

//         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//         frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//         telemetry.addData("MOTORS", "Initialized");
//     }

//     private void gyroInit() {
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

//         parameters.mode                = BNO055IMU.SensorMode.IMU;
//         parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//         parameters.loggingEnabled      = false;

//         imu.initialize(parameters);
//     }

//     @Override
//     public void init_loop() {
//         gyroLoop();
//         // arcadeLoop();
//         // telemetry.addData("IMU",
//         // controller.update();
//         // if (controller.AOnce()) {
//         //     arcadeMode = ! arcadeMode;
//         // }
//         // telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
//         // telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
//         // telemetry.update();
//     }

//     private void gyroLoop() {
//         telemetry.addData("IMU", imu.isGyroCalibrated() ? "Initialized" : "Initializing...");
//         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//         telemetry.addData("ANGLE", angles);
//         telemetry.addData("HEADING", angles.firstAngle);
//     }

//     private void arcadeLoop() {
//         arcadeMode = gamepad1.a;
//         telemetry.addData("ARCADE", arcadeMode ? "ACTIVE" : "OFF");
//     }

//     @Override
//     public void loop() {
//         gyroLoop();
//         setPower();
//         // arcadeLoop();
//         // controller.update();
//         // robot.loop();

//         // if (controller.XOnce()) {
//         //     robot.resetHeading();
//         // }
//         // if (controller.AOnce()) {
//         //     arcadeMode = !arcadeMode;
//         // }
//         // telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
//         // telemetry.addData("Heading (reset: x)", robot.getHeadingDegrees());
//         // telemetry.update();

//         // final double x = Math.pow(controller.left_stick_x, 3.0);
//         // final double y = Math.pow(controller.left_stick_y, 3.0);

//         // final double rotation = Math.pow(controller.right_stick_x, 3.0);
//         // final double direction = Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0);
//         // final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

//         // final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
//         // final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
//         // final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
//         // final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

//         // robot.setMotors(lf, lr, rf, rr);
//     }

//     private void setPower() {
//         final double rotation = -Math.pow(gamepad1.left_stick_x, 3.0);
//         final double y = Math.pow(gamepad1.left_stick_y, 3.0);
//         final double x = -Math.pow(gamepad1.right_stick_x, 3.0);
//         // final double rotation = -gamepad1.left_stick_x;
//         // final double y = gamepad1.left_stick_y;
//         // final double x = -gamepad1.right_stick_x;



//         final float currHeading = angles.firstAngle;
//         final double direction = Math.atan2(x, y) + (arcadeMode ? currHeading : 0.0);
//         final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

//         final float brake = gamepad1.left_bumper ? 0.4f : 1.0f;

//         final double lf = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * brake;
//         final double rf = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * brake;
//         final double lr = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * brake;
//         final double rr = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * brake;

//         // telemetry.addData("ROTATION", format.format(rotation));
//         // telemetry.addData("DIRECTION", format.format(direction));
//         // telemetry.addData("SPEED", format.format(speed));

//         // telemetry.addData("LF", format.format(lf));
//         // telemetry.addData("RF", format.format(rf));
//         // telemetry.addData("LR", format.format(lr));
//         // telemetry.addData("RR", format.format(rr));

//         telemetry.addData("frontLeftPos", frontLeft.getCurrentPosition());
//         telemetry.addData("rearLeftPos", rearLeft.getCurrentPosition());
//         telemetry.addData("frontRightPos", frontRight.getCurrentPosition());
//         telemetry.addData("rearRightPos", rearRight.getCurrentPosition());

//         frontLeft.setPower(lf);
//         frontRight.setPower(rf);
//         rearLeft.setPower(lr);
//         rearRight.setPower(rr);

//     }

// }
