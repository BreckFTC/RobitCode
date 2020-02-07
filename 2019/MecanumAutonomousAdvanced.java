// mecanum control code
// Jay Jasper, 2019
// Key features
//      boost mode: choose between maximum speed and best joystick linearity
//      user-centric mode: if provided a robot heading, can give commands in world frame (vs. local robot frame)
//      frame selection: command robot rotation movement about arbitrary points (e.g. the end effector)

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumAutonomousAdvanced {

    //private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private DcMotorEx[] wheel_motors; // [4]
    private int i; // for loop index for going over wheel motors

    // Wheel order:
    // 0 - front left
    // 1 - front right
    // 2 - rear left
    // 3 - rear right

    // wheel locations TODO robot geometry
    double wheel_x = 0.085;
    double wheel_y = 0.170;
    private double[][] wheel_loc = {{wheel_x, -wheel_y},
                    {wheel_x, wheel_y},
                    {-wheel_x, -wheel_y},
                    {-wheel_x, wheel_y}};

    // which direction does the wheel push the robot when spun forwards?
    private double c45 =  Math.sqrt(2.0)/2.0;
    private double[][] wheel_push = {{c45, c45}, {c45, -c45}, {c45, -c45}, {c45, c45}};

    private double MAX_DRIVE_SPEED = 2480f;
    private double mctrl_f = 32767 / MAX_DRIVE_SPEED; // "unit effort" / (tic/second)
    private double mctrl_p = 1 * 0.6 * mctrl_f; // using 0.6*2 seemed better on the ground?
    private double mctrl_i = 1.5 * 0.5 * mctrl_f; // using 0.5*2 seemed better on the ground?
    private double mctrl_d = 0.1 * mctrl_p;

    // max wheel tangent speed, m/s
    public double TIC_PER_REV = 480; // confirmed by simple experiment
    public double WHEEL_DIAMETER = 0.095; // m
    public double MAX_WHEEL_SPEED =  MAX_DRIVE_SPEED * Math.PI * WHEEL_DIAMETER / TIC_PER_REV; // 0.491 m/s
    public double MAX_ROTATE = Math.sqrt(2)*MAX_WHEEL_SPEED / (Math.sqrt(Math.pow(wheel_loc[0][0],2) + Math.pow(wheel_loc[0][1],2)));

    private double[] desired_v_global; // [3] x, y, rotation
    private double[] desired_v; // [3] x, y, rotation after usercentric transform
    private double[] v_local; // [2] temp var x,y local at each wheel
    private double[] w; // [4] velocity of each wheel!
    private double th; // TODO user centric angle
    private double effort_current; // boost mode working variable
    private double limit_den; // boost mode working variable
    private double effort_max; // boost mode working variable
    private double effort_lvl; // bost mode working variable
    private double w_max; // boost mode working variable

    private double[] center_coords; // [2] x,y

    private float target_slide;
    private float target_rotate;
    private float target_fwd;

    // init function
    public void init(Gamepad gamepad, HardwareMap hardwareMap){
        // take joystick item
        gamepad1 = gamepad;

        // get motor references
        wheel_motors = new DcMotorEx[4];
        wheel_motors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeft");
        wheel_motors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRight");
        wheel_motors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rearLeft");
        wheel_motors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rearRight");

        // set motor properties
        for (i=0; i<4; i++){
            wheel_motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel_motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel_motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel_motors[i].setVelocityPIDFCoefficients(mctrl_p, mctrl_i, mctrl_d, mctrl_f);
        }
        wheel_motors[0].setDirection(DcMotor.Direction.REVERSE);
        wheel_motors[2].setDirection(DcMotor.Direction.REVERSE);

        // initialize variables once to try and avoid GC losses
        desired_v_global = new double[3];
        desired_v = new double[3];
        v_local = new double[2];
        w = new double[4];
        center_coords = new double[2];
        th = 0; // TODO gyro stuff

        setCenterCoords("robot");
    }

    // get IC from string name
    // TODO a better formulation for live updating due to gripper height?
    public void setCenterCoords(String name){
        switch(name){
            case "fwdblock":
                center_coords[0] = 0.440;
                center_coords[1] = 0.0;
                break;
            case "0block":
                center_coords[0] = 0.25;
                center_coords[1] = 0;
                break;
            case "foundation":
                center_coords[0] = -0.600;
                center_coords[1] = 0;
                break;
            case "robot":
            default:
                center_coords[0] = 0;
                center_coords[1] = 0;
        }
    }

    // go function
    public void go(){
        // get the desired vector from joystick
        desired_v_global[0] = MAX_WHEEL_SPEED * target_fwd;
        desired_v_global[1] = MAX_WHEEL_SPEED * target_slide;
        desired_v_global[2] = MAX_ROTATE * target_rotate;

        // TODO rotate if user centric mode
        desired_v[0] = desired_v_global[0]*Math.cos(th) + desired_v_global[1]*Math.sin(th);
        desired_v[1] = -desired_v_global[0]*Math.sin(th) + desired_v_global[1]*Math.cos(th);
        desired_v[2] = desired_v_global[2];

        // center/frame selection
        if (joystick_center_robot()){
            setCenterCoords("robot");
        }
        if (joystick_center_gripper()){
            setCenterCoords("foundation");
        }
        // if (joystick_center_foundation()){
        //     setCenterCoords("fwdblock");
        // }

        for (i=0; i<4; i++){
            // get local velocity at each wheel. translation plus cross product
            v_local[0] = desired_v[0];
            v_local[1] = desired_v[1];
            v_local[0] += -desired_v[2]*(wheel_loc[i][1] - center_coords[1]);
            v_local[1] += desired_v[2]*(wheel_loc[i][0] - center_coords[0]);
            // dot product to get wheel contribution. in m/s here!
            w[i] = wheel_push[i][0]*v_local[0] + wheel_push[i][1]*v_local[1];
        }


        // handle boost mode
        if (joystick_boost()){
            // a nonlinear mapping to maximize speed
            // multiply wheel speeds so the maximum magnitude is 1?
            // then scale by the effort percentage
            effort_current = Math.sqrt(Math.pow(target_fwd,2) + Math.pow(target_slide,2) + Math.pow(target_rotate,2));

            // get max possible length of the joystick vector
            limit_den = max3(Math.abs(target_fwd), Math.abs(target_slide), Math.abs(target_rotate));
            effort_max = Math.sqrt(Math.pow(target_fwd / limit_den,2) + Math.pow(target_slide / limit_den,2) + Math.pow(target_rotate / limit_den,2));

            // scale desired twist by this - so joystick at the edge always gets SOME motor to full velocity
            effort_lvl = effort_current / effort_max;

            w_max = max4(Math.abs(w[0]), Math.abs(w[1]), Math.abs(w[2]), Math.abs(w[3]));
            for (i=0; i<4; i++){
                w[i] *= MAX_WHEEL_SPEED * effort_lvl / w_max;
            }

        }
        else{
            for (i=0; i<4; i++){
                w[i] /= Math.sqrt(2);
            }
        }


        for (i=0; i<4; i++){
            // convert m/s to tic/s
            w[i] *= TIC_PER_REV / (Math.PI*WHEEL_DIAMETER);

            // then send command to the motor
            wheel_motors[i].setVelocity(w[i]);
        }

        //telemetry.addData("wheel[0] speed", wheel_motors[0].getVelocity());
        // telemetry.addData("wheel_motors[1] Enc", wheel_motors[1].getCurrentPosition());

    }
    float max3(float a, float b, float c){
        return Math.max(Math.max(a,b), c);
    }
    double max4(double a, double b, double c, double d){
        return Math.max(Math.max(a,b), Math.max(c,d));
    }

    public int getFrontRight(){
        return wheel_motors[1].getCurrentPosition();
    }

    public void setSlide(float slide){
      target_slide = -slide * Math.abs(slide);
    }

    public void setFwd(float fwd){
      target_fwd = fwd * Math.abs(fwd);
    }

    public void setRotate(float rotate){
      target_rotate = -rotate * Math.abs(rotate);
    }

    // put joystick reads as functions so remapping is easy
    float joystick_fwd(){ // also put joystick curves here
        return gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
    }
    float joystick_side(){ // also put joystick curves here
        return -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
    }
    float joystick_rotate(){ // also put joystick curves here
        return -0.5f*gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
    }
    boolean joystick_boost(){
        return gamepad1.right_bumper;
    }
    boolean joystick_center_robot(){
        return gamepad1.dpad_down;
    }
    boolean joystick_center_gripper(){
        return gamepad1.dpad_right;
    }
    boolean joystick_center_foundation(){
        return gamepad1.dpad_up;
    }
    boolean joystick_user_centric(){
        return gamepad1.back;
    }
    boolean joystick_robot_centric(){
        return gamepad1.start;
    }
}

