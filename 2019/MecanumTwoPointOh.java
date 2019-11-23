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
// import com.qualcomm.robotcore.hardware.Servo;
// import java.text.DecimalFormat;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;
// import java.text.SimpleDateFormat;
// import java.util.Date;

// /**
//  * This file contains an example of an iterative (Non-Linear) "OpMode".
//  * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
//  * The names of OpModes appear on the menu of the FTC Driver Station.
//  * When an selection is made from the menu, the corresponding OpMode
//  * class is instantiated on the Robot Controller and executed.
//  *
//  * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
//  * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
//  */
// @TeleOp

// public class MecanumTwoPointOh extends OpMode {
//     public DecimalFormat format = new DecimalFormat("##.00");
//     private BNO055IMU imu;
//     private DcMotor frontLeft;
//     private DcMotor rearLeft;
//     private DcMotor frontRight;
//     private DcMotor rearRight;
//     private DcMotor manArm;

//     private Servo elbow;
//     private Servo hand;

//     public double leftStickY;
//     public double leftStickX;
//     public double rightStickX;

//     public float gyro_degrees;

//     public double newForward;
//     public double newStrafe;
//     public double newRotation;

//     // State used for updating telemetry
//     public Orientation angles;
//     public Acceleration gravity;

//     private boolean arcadeMode = false;
//     private int gyroCalibratedCount = 0;
//     public double scalingFactor = 1;

//     @Override
//     public void init() {
//         motorInit();
//         gyroInit();
//         gamepadInit();
//     }

//     private void gamepadInit() {
//         gamepad1.setJoystickDeadzone(0.2f);
//     }

//     private void motorInit() {
//         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//         rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
//         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//         rearRight = hardwareMap.get(DcMotor.class, "rearRight");

//         manArm = hardwareMap.get(DcMotor.class, "manArm");

//         elbow = hardwareMap.get(Servo.class, "leElbo");
//         hand = hardwareMap.get(Servo.class, "leHand");


//         frontLeft.setDirection(DcMotor.Direction.REVERSE);
//         rearRight.setDirection(DcMotor.Direction.REVERSE);

//         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         manArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
//         setDrivePower();
//         setArmPosition();
//         setElbowPower();
//         setHandPower();
//     }

//     private void setDrivePower() {
//         final double rotation = -Math.pow(gamepad1.right_stick_x, 3.0) * -1;
//         final double y = Math.pow(gamepad1.left_stick_y, 3.0) * -1;
//         final double x = -Math.pow(gamepad1.left_stick_x, 3.0) * -1;

//         final float currHeading = angles.firstAngle;
//         final double direction = Math.atan2(x, y) + (arcadeMode ? currHeading : 0.0);
//         final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

//         final float brake = gamepad1.left_bumper ? 0.4f : 1.0f;

//         final double lf = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
//         final double rf = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;
//         final double lr = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
//         final double rr = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;

//         frontLeft.setPower(-lf);
//         frontRight.setPower(-rf);
//         rearLeft.setPower(lr);
//         rearRight.setPower(rr);
//     }

//     private void setElbowPower(){
//         telemetry.addData("elbow position", elbow.getPosition());
//         if(gamepad1.x) {
//             elbow.setPosition(0.8);
//         } else if(gamepad1.a) {
//             elbow.setPosition(0.2);
//         }
//     }

//     private void setHandPower(){
//         telemetry.addData("hand position", hand.getPosition());
//         // if(gamepad1.left_bumper) {
//         //     hand.setPosition(0.6);
//         // } else if(gamepad1.right_bumper) {
//         //     hand.setPosition(0);
//         // }
//     }

//     private void setArmPosition() {
//         telemetry.addData("manArmPos", manArm.getCurrentPosition());
//         manArm.setPower(0.75);
//         if(gamepad1.right_bumper) {
//             manArm.setTargetPosition(-250);
//             manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         } else if (gamepad1.left_bumper) {
//             manArm.setTargetPosition(0);
//             manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         }
//         // final double manArmPower = (gamepad1.left_trigger * -1) + gamepad1.right_trigger;
//         // telemetry.addData("manArmPower", manArmPower);
//         // manArm.setPower(manArmPower*0.5);
//     }
// }
