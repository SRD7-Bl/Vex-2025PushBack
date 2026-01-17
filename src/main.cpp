#include "field_map.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-3, -6, -10}, pros::MotorGearset::blue); // left motor group - ports -3，-2，-6
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 14,15,16
pros::Motor intake_motor(19,pros::MotorGearset::blue); //intake motor
pros::Motor outfeed_motor(8,pros::MotorGearset::blue); //outfeed motor
pros::Distance intake_distance_sensor(9); //distance sensor
pros::Distance outfeed_distance_sensor(1);
pros::adi::DigitalOut valveA('A');
pros::adi::DigitalOut valveB('B');
pros::adi::DigitalOut valveH('H');

// Inertial Sensor on port 10
pros::Imu imu(7);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed

pros::Rotation horizontalEnc(4);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(5);

// horizontal tracking wheel. 1.8" diameter, 0.5" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, 1);

// vertical tracking wheel. 1.8" diameter, 1.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2, 1.25); 

// 2 ，-1
// 2 ， 1.25
// 1.9 , -0.5
// 1.9 , -1.5


// 16.5*14
// 8.25*7

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 14 inch track width
                              3, // using new 3" omnis
                              600, // drivetrain rpm is 600(We use blue motor)
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
//12.5 9 30
lemlib::ControllerSettings linearController(9, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            30, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
//5.5 0 60
lemlib::ControllerSettings angularController(5.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             60, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/*
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

bool g_isBlue = false;
bool Isright  = false;

// 场地坐标（你测量的）
const double Xf_red  = 8.25;
const double Yf_red  = 54.3;
const double Thf_red = 0;   // 先保留你现在的朝向

const double Xf_blue = 132.18;
const double Yf_blue = 86.91;
const double Thf_blue = 180.0;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::lcd::print(0, "init, chassis=%p", &chassis);

    horizontalEnc.reset_position();
    verticalEnc.reset_position();

    chassis.calibrate(); // calibrate sensors   
    pros::delay(100);

    if(Isright){
        if (!g_isBlue) {
            chassis.setPose(Xf_red, Yf_red, 90);
        } else {
            chassis.setPose(Xf_blue, Yf_blue, -90);
        }
    }
    else{
        if(!g_isBlue){
            chassis.setPose(Xf_red,Yf_blue,90);
        }
        else{
            chassis.setPose(Xf_blue,Yf_red,-90);
        }
    }
    

}

const int INTAKE_SPEED = 1000;
const int OUTF_EED_SPEED = 1000;
void runIntakeForMs(int ms) {
    // TODO: 换成你真实的电机对象/函数
    intake_motor.move(11000);  // 或 move_voltage(...)
    pros::delay(ms);
    intake_motor.move(0);      // 停
}

// 简单版：让 outfeed 转 ms 毫秒，然后停
void runOutfeedForMs(int ms) {
    // TODO: 换成你真实的电机对象/函数
    outfeed_motor.move(11000);
    pros::delay(ms);
    outfeed_motor.move(0);
}


/*
------------------------------------------------------------------------------------------------------------------------------
----------------------------------------Steering and forward module (can be used directly)------------------------------------
------------------------------------------------------------------------------------------------------------------------------
*/
inline std::pair<int,int> Error_Speed_Coarse_Mapping(double absErrorDeg){ // return {maxspeed, minspeed} (Unit: Voltage(0~127) )
    if(absErrorDeg > 90) return {85,15};
    if(absErrorDeg > 45) return {70,12};
    if(absErrorDeg > 20) return {55,11};
    if(absErrorDeg > 10) return {45,10};
    return                      {38, 8};
}

// 机器人的初始角度差又90度，这个方法负责弥补回来
double fieldAngleToOdom(double theta_field_deg) {
    return wrap_deg(-theta_field_deg + 90.0);
}
void Face_Target_Direction(double dtheta){

    if (std::isnan(dtheta)) {
        pros::lcd::print(0, 0, "Skip NaN heading");
        return;
    }
    lemlib::Pose pose = chassis.getPose(); 
    double err = wrap_deg(dtheta - pose.theta);
    double d0  = std::fabs(err);  

    auto [MaxSpeed_Coarse, MinSpeed_Coarse] = Error_Speed_Coarse_Mapping(d0);
    int t0 = (int)std::clamp(d0 * 10.0, 250.0, 900.0);

    chassis.turnToHeading(dtheta, t0,
        {.maxSpeed = MaxSpeed_Coarse,
         .minSpeed = MinSpeed_Coarse,
         .earlyExitRange = 2.0f});
    //chassis.waitUntilDone();
}

enum class FaceMode {
    NONE,           // 不管最后面向
    FACE_TARGET,    // 让“前脸”对着目标点（intake 面）
    BACK_TO_TARGET  // 让“屁股”对着目标点（outfeed 面）
};

inline double Speed_to_lem(double speed){ // the function that map the speed from M1 to Lemlib Range(0 ~ 127)
    //Tip: Speed,VMIN,and VMAX are both percentages.
    double percentage = std::clamp(speed,0.0,1.0); 
    return (float)(percentage * 127.0);
}

void Goto_with_Auxiliary_NODE(double TargetX, double TargetY,int total_ms = 2500, double speed = 0.85, double speed_minrate = 0.4,FaceMode faceMode = FaceMode::NONE){
    const float Exit_Range = 0.75f;   // 距离小于这个就认为到了

    // 1. 当前姿态（和 Face_Point_Direction 一样，直接用 chassis.getPose）
    lemlib::Pose pose = chassis.getPose();

    // 2. 计算到目标的向量（同一坐标系，不再做 fieldToOdom 和 90° 旋转）
    double dx   = TargetX - pose.x;
    double dy   = TargetY - pose.y;
    double dist = std::hypot(dx, dy);

    // 离目标太近就不动了，防止抖动
    if (dist <= Exit_Range) return;

    //controller.print(0,0,"Distance: %.2f, %.2f",intake_distance_sensor.get_distance(),outfeed_distance_sensor.get_distance());
    controller.print(0,0,"x,y:%.2f,%.2f",pose.x,pose.y);

    /*
    double phi = std::atan2(dy, dx) * 180.0 / M_PI;      // 目标全局朝向
    double err = wrap_deg(phi - pose.theta);             // 当前 heading 和目标方向的夹角
    if (std::fabs(err) > 120.0) {
        Face_Target_Direction(phi);   // 这一步你自己可控，不会莫名其妙
        pose = chassis.getPose();     // 更新一下姿态
    }
    */

    bool isfoward = (faceMode == FaceMode::BACK_TO_TARGET) ? false : true;

    // 3. 速度：speed 是 0~1 的比例，用 Speed_to_lem 转成 0~127
    double speed_clamped = std::clamp(speed, 0.0, 1.0);
    const float maxSpeed_lem = Speed_to_lem(speed_clamped);
    const float minSpeed_lem = maxSpeed_lem * speed_minrate;   // 给个不为 0 的最小速度防止卡死

    // 4. 直接调用 lemlib 的 moveToPoint
    chassis.moveToPoint(
        TargetX, TargetY, total_ms,
        {
            .forwards      = isfoward,
            .maxSpeed      = maxSpeed_lem,
            .minSpeed      = minSpeed_lem,
            .earlyExitRange= Exit_Range
        },
        false  // 等待这次动作完成（同步）
    );
    chassis.waitUntilDone();
    // 5. 可选：调试信息
    pros::lcd::print(1, "Goto NODE Target : %.2f, %.2f", TargetX, TargetY);
    pros::lcd::print(2, "Goto NODE Pose   : %.2f, %.2f", pose.x, pose.y);
    pros::lcd::print(3, "Goto NODE Dist   : %.2f",       dist);
}

void Face_Point_Direction(double TargetX,double TargetY){
    
     // 1. 当前姿态（假定 chassis.getPose() 的 x,y,theta 全部是在同一个“odom坐标系”）
    lemlib::Pose pose = chassis.getPose();

    // 3. 计算 odom 坐标系下的向量
    double dx = TargetX - pose.x;
    double dy = TargetY - pose.y;

    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return; // 就在脚边，不转

    // 4. 直接用 atan2 得到“目标绝对朝向”phi（odom 坐标系）
    double phi = std::atan2(dy, dx) * 180.0 / M_PI;
    //phi -= 90.0; //旋转坐标系回调
    double targetHeading = wrap_deg(-phi + 90);

    // 日志：当前角度、希望的 phi，以及误差
    double err = wrap_deg(targetHeading - wrap_deg(pose.theta));
    controller.print(0,0,"%.2f,%.2f,%.2f", wrap_deg(pose.theta), phi, targetHeading);

    // 5. 交给前面简化后的 Face_Target_Direction 来执行转向
    Face_Target_Direction(targetHeading);

    pros::delay(20);
}




/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {

    /*
    pros::lcd::initialize(); // initialize brain screen
    pros::lcd::print(0, "init, chassis=%p", &chassis);

    horizontalEnc.reset_position();
    verticalEnc.reset_position();

    chassis.calibrate(); // calibrate sensors   
    pros::delay(100);
    chassis.setPose(68.5,23,270);
    SKILL();
    */
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


// three functions to find the coordinate, angle in the struct.


// extremely easy module, used for emergency autonomous script
void drive_arcade_ms(int forward, int turn, int ms) {
    chassis.arcade(forward, turn, true); // linear
    pros::delay(ms);
    chassis.arcade(0, 0, true);
}
void Turn_Relative(double deltaDeg) {
    lemlib::Pose p = chassis.getPose();
    double targetHeading = wrap_deg(p.theta + deltaDeg);

    Face_Target_Direction(targetHeading);
}

void pid_test(){
    lemlib::Pose pose = chassis.getPose();
    
    chassis.moveToPoint(pose.x+20,pose.y,2500);
    
    pros::delay(500);
}

void Emergency() {
    
    drive_arcade_ms(60, 0, 670);

    // <1> Turning
    Turn_Relative(20);

    //<2> Take a slow move while intaking 3 blocks.
    intake_motor.move_voltage(-12000);
    drive_arcade_ms(20, 0, 3500);
    pros::delay(400);
    drive_arcade_ms(-60, 0, 230); //go back a little bit
    pros::delay(400);
    intake_motor.move_voltage(0);

    // <3> Turn to the lower goal
    Turn_Relative(-65);

    // <4> Go straight
    drive_arcade_ms(80, 0, 185);

    // <5> Oufeed 3 the blocks into the lower goal.

    intake_motor.move_voltage(6000);
    pros::delay(3350);
    intake_motor.move_voltage(0);
    pros::delay(200);

    chassis.arcade(0, 0, true);


    /*
    // <6> Go to the spac point
    
    drive_arcade_ms(-80,0,600);

    drive_arcade_ms(-80,0,365);
    pros::delay(100);

    // Turn to the long goal
    Turn_Relative(-125);

    valveA.set_value(true); //open the match loader 
    pros::delay(100);
    // <8> 直走对准
    
    const int MS_DRIVE_TO_LONGGOAL_WITH2 = 275;
    drive_arcade_ms(-80,0,MS_DRIVE_TO_LONGGOAL_WITH2);
    valveB.set_value(true);
    pros::delay(100);
    // <9> 吐
    outfeed_motor.move_voltage(12000);
    intake_motor.move_voltage(-12000);
    pros::delay(2200);
    outfeed_motor.move_voltage(0);
    intake_motor.move_voltage(0);

    // <10> 去loader(无需转向)
    
    const int MS_DRIVE_TO_LOADER = 130;
    intake_motor.move_voltage(-12000);
    drive_arcade_ms(120,0,MS_DRIVE_TO_LONGGOAL_WITH2);
    pros::delay(2750);
    intake_motor.move_voltage(0);
    pros::delay(500);
    // <11> 回long goal
    const int MS_DRIVE_TO_LONGGOAL_WITH3 = 800;
    drive_arcade_ms(-80,0,MS_DRIVE_TO_LONGGOAL_WITH3);

    // <12> 吐
    valveA.set_value(true);
    outfeed_motor.move_voltage(12000);
    pros::delay(2000);
    outfeed_motor.move_voltage(0);

    // --- 收尾：停住 ---
    chassis.arcade(0, 0, true);
    */
    
}

/*
Road Map:
Plan B(最基础) -> 高级转向 -> 简单移动路线变成高级移动模块(直走，不涉及边走边捡) -> 全高级移动 -> 多线程 -> IO
如果高级移动还是会炸，可以考虑先回归IO最后整高级移动
*/

void Normal_Using() {
    // <1> 转到三连块
    drive_arcade_ms(60, 0, 670);

    if(auto c = find_coord("Center_right_red_block_right")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_right_red_block_right!");}

    
    
    //Turn_Relative(-20);
    pros::delay(1000);
    // <2> 边慢走边 intake，吃三连块
    valveB.set_value(false);
    const int MS_INTAKE_DRIVE = 1100;
    //intake_motor.move_voltage(-12000);

    drive_arcade_ms(20, 0, 3500);
    pros::delay(400);
    drive_arcade_ms(-60, 0, 500);
    /*
    drive_arcade_ms(-40, 0, 150);
    pros::delay(400);
    intake_motor.move_voltage(0);
    */
    
    // <3> 转到lower goal
    if(auto c = find_coord("LowerGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
    }else{pros::lcd::print(1,"Can not find Coordinate of LowerGoal_red_end!");}

    //Turn_Relative(85);
    
    
    // <4> 直走
    const int MS_TURN_TO_GOAL = 225; 
    drive_arcade_ms(80, 0, MS_TURN_TO_GOAL);

    // <5> 吐块: 卡时间控制吐 1-2 块
    //intake_motor.move_voltage(12000);
    pros::delay(2850);
    //intake_motor.move_voltage(0);
    pros::delay(200);

    chassis.arcade(0, 0, true);

    /*
    outfeed_motor.move_voltage(12000);
    pros::delay(2850);
    outfeed_motor.move_voltage(0);
    pros::delay(200);
    */

    // <6> 无需转向直接直走到spac point
    
    const int MS_DRIVE_TO_SPEC = 1100;
    drive_arcade_ms(-80,0,620);

    if(auto c = find_coord("Left_bottom_SpecPoint")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_bottom_SpecPoint!");}
    
    //Turn_Relative(35);

    drive_arcade_ms(80,0,420);
    pros::delay(200);
    // <7> 转到Long goal
    if(auto c = find_coord("Right_LongGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
    }else{pros::lcd::print(1,"Can not find Coordinate of Right_LongGoal_red_end!");}

    //Turn_Relative(115);

    valveA.set_value(true);
    pros::delay(500);
    // <8> 直走对准
    
    const int MS_DRIVE_TO_LONGGOAL_WITH2 = 350;
    drive_arcade_ms(-80,0,MS_DRIVE_TO_LONGGOAL_WITH2);
    valveB.set_value(true);
    pros::delay(200);

    // <9> 吐
    //outfeed_motor.move_voltage(12000);
    pros::delay(1200);
    outfeed_motor.move_voltage(0);

    // <10> 去loader(无需转向)
    
    const int MS_DRIVE_TO_LOADER = 500;
    //intake_motor.move_voltage(-12000);
    drive_arcade_ms(120,0,MS_DRIVE_TO_LONGGOAL_WITH2);
    pros::delay(2750);
    intake_motor.move_voltage(0);
    pros::delay(500);
    // <11> 回long goal
    const int MS_DRIVE_TO_LONGGOAL_WITH3 = 800;
    drive_arcade_ms(-80,0,MS_DRIVE_TO_LONGGOAL_WITH3);

    // <12> 吐
    valveA.set_value(true);
    //outfeed_motor.move_voltage(12000);
    pros::delay(2000);
    outfeed_motor.move_voltage(0);

    // --- 收尾：停住 ---
    chassis.arcade(0, 0, true);
    
}
void Experimental_WithGOING() {

    // <1> 三连块
    intake_motor.move_voltage(-12000);
    outfeed_motor.move_voltage(-8000);
    pros::delay(100);

    if(auto c = find_coord("Center_right_red_block_top")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.3,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_right_red_block_top.");}

    // 突进吃块
    drive_arcade_ms(127, 0, 150);
    pros::delay(150);
    drive_arcade_ms(-127, 0, 465);
    pros::delay(400);

    intake_motor.move_voltage(0);
    outfeed_motor.move_voltage(0);
    pros::delay(100);

    // <2> Lower Goal
    if(auto c = find_coord("LowerGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2050,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of LowerGoal_red_end");}
    
    // <3> 吐块: 卡时间控制吐 1-2 块
    intake_motor.move_voltage(12000);
    pros::delay(1150);
    intake_motor.move_voltage(0);
    pros::delay(100);

    // <4> Back
    //drive_arcade_ms(-127, 0, 205);
    pros::delay(100);

    /*
    if(auto c = find_coord("Center_right_red_block_top")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.9,0.65,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_right_red_block_top.");}
    */
    
    
    if(auto c = find_coord("Left_bottom_SpecPoint")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //drive_arcade_ms(-127, 0, 300);
        pros::delay(700);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,4700,0.6,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_bottom_SpecPoint!");}
    valveA.set_value(true);
    valveB.set_value(true);
    pros::delay(550);

    // <7> 直接到loader
    valveA.set_value(true);
    valveB.set_value(true);
    intake_motor.move_voltage(-11000);
    outfeed_motor.move_voltage(-8000);
    pros::delay(50);

    if(auto c = find_coord("Red_right_loader")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.6,0.75);
    }else{pros::lcd::print(1,"Can not find Coordinate of Red_right_loader!");}

    pros::delay(50);
    intake_motor.move_voltage(0);
    outfeed_motor.move_voltage(0);

    // <8> 去long goal
    
    if(auto c = find_coord("Right_LongGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.55,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Right_LongGoal_red_end!");}
    
    /*
    drive_arcade_ms(-127,0,450);
    pros::delay(1000);
    */
   
    outfeed_motor.move_voltage(11500);
    intake_motor.move_voltage(-12000);
    pros::delay(2200);
    outfeed_motor.move_voltage(0);
    intake_motor.move_voltage(0);
    
    drive_arcade_ms(127, 0, 200);
    pros::delay(200);
    drive_arcade_ms(-127, 0, 350);
    pros::delay(700);

    // --- 收尾：停住 ---
    chassis.arcade(0, 0, true);

}

void Experimental_WithGOING2() {
    // <1> 三连块
    intake_motor.move_voltage(-12000);
    outfeed_motor.move_voltage(-9000);
    drive_arcade_ms(127, 0, 250);
    pros::delay(100);

    if(auto c = find_coord("Center_left_red_block_right")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.4,0.6);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    // 突进吃块
    
    
    drive_arcade_ms(127, 0, 100);
    pros::delay(50);
    drive_arcade_ms(-127, 0, 180);
    //pros::delay(100);

    /*
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(pose.x-10,pose.y,1000,{.forwards=false});
    */

    intake_motor.move_voltage(0);
    outfeed_motor.move_voltage(0);
    //valveA.set_value(true);
    pros::delay(50);

    // <2> Lower Goal
    /*
    if(auto c = find_coord("UpperGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,7050,0.55,0.35,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of UpperGoal_red_end");}
    
    // <3> 吐块: 卡时间控制吐 1-2 块
    valveA.set_value(false);
    pros::delay(100);
    intake_motor.move_voltage(-8000);
    outfeed_motor.move_voltage(8000);
    pros::delay(1150);
    intake_motor.move_voltage(0);
    outfeed_motor.move_voltage(0);
    pros::delay(100);
    */

    // <4> Back
    //drive_arcade_ms(127, 0, 200);
    //pros::delay(200);
    valveB.set_value(true);

    /*
    if(auto c = find_coord("Center_right_red_block_top")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.9,0.65,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_right_red_block_top.");}
    */
    
    outfeed_motor.move_voltage(-5000);
    pros::delay(100);

    if(auto c = find_coord("Left_top_SpecPoint")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //drive_arcade_ms(-127, 0, 300);
        pros::delay(500);
        Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,1700,0.7,0.55);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_top_SpecPoint!");}
    
    //Face_Target_Direction(fieldAngleToOdom(180));

    // <7> 直接到loader
    valveA.set_value(true);
    valveB.set_value(true);
    intake_motor.move_voltage(-11000);
    outfeed_motor.move_voltage(-8000);
    pros::delay(100);

    if(auto c = find_coord("Red_left_loader")){
        auto p = transform_for_alliance(*c, g_isBlue);
        Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2000,0.4,0.25);
    }else{pros::lcd::print(1,"Can not find Coordinate of Red_left_loader!");}

    pros::delay(400);
    intake_motor.move_voltage(0);
    outfeed_motor.move_voltage(0);

    //Face_Target_Direction(fieldAngleToOdom(0));
    // <8> 去long goal
    
    
    if(auto c = find_coord("Left_LongGoal_red_end")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,1700,0.6,0.5,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_LongGoal_red_end!");}
    //pros::delay(300);
    
    /*
    Face_Target_Direction(fieldAngleToOdom(180));
    pros::delay(100);

    drive_arcade_ms(-127,0,50);
    pros::delay(100);

    Face_Target_Direction(fieldAngleToOdom(180));
    pros::delay(100);

    drive_arcade_ms(-127,0,350);
    pros::delay(600);
    */
    
    valveA.set_value(true);
    outfeed_motor.move_voltage(10500);
    intake_motor.move_voltage(-10000);
    pros::delay(2200);
    outfeed_motor.move_voltage(0);
    intake_motor.move_voltage(0);
    
    drive_arcade_ms(127, 0, 300);
    //valveH.set_value(true);
    valveB.set_value(false);
    pros::delay(100);
    
    
    if(auto c = find_coord("Left_Descore_point")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.6,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_LongGoal_red_end!");}

    Face_Target_Direction(fieldAngleToOdom(0));
    pros::delay(200);

     if(auto c = find_coord("Left_Descore_point2")){
        auto p = transform_for_alliance(*c, g_isBlue);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.6,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Left_LongGoal_red_end!");}


    /*
    drive_arcade_ms(127,0,175);
    pros::delay(300);
    */
    // --- 收尾：停住 ---
    
}

/*

Intake motor : negative -> intake ; positive -> go out from the bottom.
Outfeed motor : negative -> go back ; positive -> go out.

*/
void Intake_block(std::string Command = "Start",float rate = 1.0f){
    if(Command == "Start"){
        intake_motor.move_voltage(-12000 * rate);
        outfeed_motor.move_voltage(-8000 * rate);
        pros::delay(100);
    } else if (Command == "Stop"){
        intake_motor.move_voltage(0);
        outfeed_motor.move_voltage(0);
        pros::delay(100);
    }
}
void OutFeed_block(std::string Command = "Start", float rate = 1.0f){
    if (Command == "Start"){
        intake_motor.move_voltage(-12000 * rate);
        outfeed_motor.move_voltage(12000 * rate);
        pros::delay(100);
    } else if (Command == "Stop"){
        intake_motor.move_voltage(0);
        outfeed_motor.move_voltage(0);
        pros::delay(100);
    }
}

void SKILL(){
    
    //Right half
    if(auto p = find_coord("Left_bottom_SpecPoint")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block();

    if(auto p = find_coord("Red_right_loader")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}
    pros::delay(750);

    Intake_block("Stop");

    if(auto p = find_coord("Center_right_red_block_center")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    if(auto p_raw = find_coord("Center_left_red_block_center")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    if(auto p_raw = find_coord("Left_top_SpecPoint")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    if(auto p_raw = find_coord("Left_LongGoal_red_end")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    OutFeed_block();
    pros::delay(1750);

    OutFeed_block("Stop");
    Intake_block();
    pros::delay(100);
    
    if(auto p_raw = find_coord("Red_left_loader")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}
    pros::delay(750);

    if(auto p_raw = find_coord("Left_LongGoal_red_end")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block("Stop");
    OutFeed_block();
    pros::delay(1750);

    OutFeed_block("Stop");

    drive_arcade_ms(127,0,150);
    pros::delay(100);

    /*
    if(auto p = find_coord("Right_SidePoint_BlueSide")){
        //auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block();

    if(auto p_raw = find_coord("Parking_blue_block_left4")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block("Stop");
    */

    //left half

    if(auto p_raw = find_coord("Left_bottom_SpecPoint")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block();
    
    if(auto p_raw = find_coord("Red_right_loader")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block("Stop");

    if(auto p_raw = find_coord("Right_LongGoal_red_end")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    OutFeed_block();
    pros::delay(1750);
    OutFeed_block("Stop");

    drive_arcade_ms(127,0,150);
    pros::delay(100);

    if(auto p_raw = find_coord("Center_right_red_block_center")){
        auto p = transform_for_alliance(*p_raw, true);
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p.x_co,p.y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    if(auto p = find_coord("Center_left_red_block_center")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block();
    if(auto p = find_coord("Red_left_loader")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}
    pros::delay(750);
    Intake_block("Stop");

    if(auto p = find_coord("Left_LongGoal_red_end")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45,FaceMode::BACK_TO_TARGET);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    OutFeed_block();
    pros::delay(1750);
    OutFeed_block("Stop");

    //
    /*
    if(auto p = find_coord("Left_SidePoint_RedSide")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}

    Intake_block();
    if(auto p = find_coord("Parking_blue_block_left4")){
        //Face_Point_Direction(p.x_co,p.y_co);
        Goto_with_Auxiliary_NODE(p->x_co,p->y_co,2500,0.5,0.45);
    }else{pros::lcd::print(1,"Can not find Coordinate of Center_left_red_block_bottom.");}
    Intake_block("Stop");
    */
}

void autonomous() {

    
    pros::lcd::print(1, "auto chassis=%p", &chassis);

    //pid_test();
    //SKILL();
    /*
    Plan B: Emergency using
    */
    //Emergency();    

    /*
    Plan A: Best situation:
    */

    //Normal_Using();
    //Moving_test();
    
    
    if(Isright){
        Experimental_WithGOING();
    } else{
        Experimental_WithGOING2();
    }
    
    
}   
/*
    R1: intake 但不吐出
    R2: intake吐出 
    L1: 反转下面吐出 
    L2: 换方向 
    X: match loader pneumatics
    Y：pneumatics 换高度
 */
void opcontrol() {
    // controller
    bool OutFeedHeight          = false;
    bool MatchLoader            = false;
    bool driveReversed          = false;
    bool InputOutputReversed    = true;
    bool CRAW                   = false;
    bool CorrdORANgle           = false;
    bool SlowMode               = false;

    float IOspeed                 = 1.0;


    valveA.set_value(false);
    valveB.set_value(false);
    valveH.set_value(false);
    auto deadzone = [](int v, int dead_area = 8){return (std::abs(v) < dead_area) ? 0 : v;};    
    while (true) { // loop to continuously update motors
        //Key:Y Change outfeed height
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            OutFeedHeight = !OutFeedHeight;
            valveA.set_value(OutFeedHeight);
        }
        //Key:X match loader
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            MatchLoader = !MatchLoader;
            valveB.set_value(MatchLoader);
        }
        //Key:A Lower the claw used to reach the long goal.
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            CRAW = !CRAW;
            valveH.set_value(CRAW);
        }
        //Key:L2 Base Motor Reverse
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            driveReversed = !driveReversed;
        }
        //Key:L1 Reverse the rotation direction of intake and outfeed
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            InputOutputReversed = !InputOutputReversed;
        }
        //Key:B Slow mode
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            SlowMode = !SlowMode;
            IOspeed = (SlowMode) ? 0.85 : 1.0;
        }
        /*
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            if (!g_isBlue) {
                chassis.setPose(Xf_red, Yf_red, 90);
            } else {
                chassis.setPose(Xf_blue, Yf_blue, -90);
            }
        }
        */
        
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            CorrdORANgle = !CorrdORANgle;
        }
        
        
        auto p = chassis.getPose();
        if(CorrdORANgle){
            controller.print(0, 0, "%.2f, %.2f",   p.x,p.y);
        }else{
            controller.print(0, 0, "%.2f,%.2f", wrap_deg(p.theta), wrap_deg(p.theta - 90));
        }
        //auto Odom_p = OdomTofield(p.x,p.y,p.theta);
        //controller.print(0, 0, "X:%.2f, Y:%.2f", Odom_p.x, Odom_p.y);
        //controller.print(0, 0, "%.2f, %.2f       ",   Odom_p.x, Odom_p.y);


        const int dir       = driveReversed       ? -1 : 1;
        const int dir_INOUT = InputOutputReversed ? -1 : 1;
        // R1 and R2
        const bool R1=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool R2=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        if (R1 && !R2) {
            intake_motor.move_voltage(12000 * dir_INOUT * IOspeed);
            outfeed_motor.move_voltage(9000 * dir_INOUT * IOspeed);
        } else if (!R1 && R2) {
            intake_motor.move_voltage(12000 * dir_INOUT * IOspeed);
            outfeed_motor.move_voltage(-11500 * dir_INOUT * IOspeed);
        } else {
            intake_motor.move_voltage(0);
            outfeed_motor.move_voltage(0);
        }
        int forward = deadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int turning = deadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        forward = std::clamp(forward * dir, -127, 127);
        turning = std::clamp(turning, -127, 127);
        if(turning) forward *= 0.75;
        chassis.arcade(forward,turning);
        pros::delay(20);
    }
        
}