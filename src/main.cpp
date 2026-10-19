#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motors
// Left side: Port 1 reversed, Port 2 reversed
pros::MotorGroup left_motors({-1, -2}, pros::v5::MotorGears::blue); 
// Right side: Port 10, Port 12
pros::MotorGroup right_motors({10, 12}, pros::v5::MotorGears::blue);
// Intake motors
pros::Motor intake_motor(-6, pros::v5::MotorGears::green);
// Pneumatics -> | A - PORT, False - default state (retracted)
pros::adi::Pneumatics clamp('A', false);
// Elevator motors - ADJUST ports and gearset
// Full Motors Group
pros::MotorGroup full_motors({17, 20}, pros::v5::MotorGears::green);
// Half Motors Group 
pros::MotorGroup half_motors({18, -19}, pros::v5::MotorGears::green);
// IMU
// pros::Imu imu(0);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// all null for now without tracking wheels/imu
// see tracking wheels section -> https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html#odometry
lemlib::OdomSensors sensors(
    nullptr, nullptr, nullptr, nullptr, nullptr
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	
	// calibration is commented since no imu right now
    // chassis.calibrate(); // calibrate sensors

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

    // Set brake mode for motor groups
    // full_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // half_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

// SIMPLE PARKING AUTON
void parkingSkills() {
    chassis.setPose(48, 0, 0); 
    chassis.moveToPoint(0, 0, 2000, {.maxSpeed = 70});
    chassis.waitUntilDone();
}

// PRELOAD SCORE AUTON
void preloadScoreSkills() {
    clamp.set_value(true); // Set to true to hold preload

    // Initial Position -> Adjust coordinates 
    chassis.setPose(60, 0, 0); 

    // Drive forward
    chassis.moveToPoint(55, 0, 1000, {.forwards = false, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive back into toggle
    chassis.moveToPoint(60, 0, 800, {.forwards = true, .maxSpeed = 70}); 
    chassis.waitUntilDone();

    // Drive forward 
    chassis.moveToPoint(55, 0, 1000, {.forwards = false, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive back into toggle
    chassis.moveToPoint(60, 0, 1000, {.forwards = true, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive forward to align with goal
    chassis.moveToPoint(48, 0, 1000, {.forwards = true, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Turn 90 degrees counterclockwise
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();

    // Move to scoring position
    chassis.moveToPoint(48, 18, 1000, {.forwards = true, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Open/release the claw to score the preloaded element
    clamp.set_value(false); 
    pros::delay(300);

    // Drive to park
    chassis.moveToPoint(5, 0, 1000, {.forwards = true, .maxSpeed = 80}); 
    chassis.waitUntilDone();
}

// 3 Pin auton
void complexSkills() {
    clamp.set_value(true); // Set to true to hold preload

    // Initial Position -> Adjust coordinates 
    chassis.setPose(55, 0, 0); 

    // Drive forward into toggle 
    chassis.moveToPoint(60, 0, 1000, {.forwards = false, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive back 
    chassis.moveToPoint(52, 0, 800, {.forwards = true, .maxSpeed = 70}); 
    chassis.waitUntilDone();

    // Drive forward into toggle
    chassis.moveToPoint(60, 0, 1000, {.forwards = false, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive back
    chassis.moveToPoint(55, 0, 1000, {.forwards = true, .maxSpeed = 80}); 
    chassis.waitUntilDone();

    // Drive to next point
    chassis.turnToHeading(45, 800); 
    chassis.waitUntilDone();

    // Drive forward to the target goal coordinates
    chassis.moveToPoint(24, 24, 1200, {.maxSpeed = 80});
    chassis.waitUntilDone();

    // Open/release the claw to score the preloaded element
    clamp.set_value(false); 
    pros::delay(300);

    // TURN TO HEADING 90 DEGREES
    chassis.turnToHeading(90, 800); 
    chassis.waitUntilDone();

    // ALIGN WITH GOAL AND CONE
    chassis.moveToPoint(24, 28, 1200, {.maxSpeed = 80});
    chassis.waitUntilDone();

    // HEADING 0 DEGREES
    chassis.turnToHeading(0, 800); 
    chassis.waitUntilDone();

    // RUN INTAKE
    intake_motor.move(127);

    // TURN AROUND
    chassis.turnToHeading(180, 800); 
    chassis.waitUntilDone();

    // SCORE CONE
    chassis.turnToHeading(90, 800); 
    chassis.waitUntilDone();

    // MOVE DOWN
    chassis.moveToPoint(24, -48, 1200, {.maxSpeed = 80});
    chassis.waitUntilDone();

    // TURN TO FACE YELLOW PIN
    chassis.turnToHeading(0, 800); 
    chassis.waitUntilDone();

    // RUN INTAKE
    intake_motor.move(127);

    // REVERSE
    chassis.turnToHeading(180, 800); 
    chassis.waitUntilDone();

    // SCORE

    // PARKING
    chassis.moveToPoint(0, 0, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // Uncomment the desired autonomous routine
    // parkingSkills();
    // preloadScoreSkills();
    // complexSkills();
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    /*
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
        // Arcade control scheme
        int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
        left_mg.move(dir - turn);                      // Sets left motor voltage
        right_mg.move(dir + turn);                     // Sets right motor voltage
        pros::delay(20);                               // Run for 20 ms then update
    }
    */
    bool armrunningu=false;
    bool armrunningd=false; 
    bool elevator_runu=false; 
    bool elevator_rund=false;
    bool piston_extended=false;

    while (true) {
        // Left stick UP/DOWN -> throttle 
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // Right stick LEFT/RIGHT -> turn 
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // arcade drive
        chassis.arcade(throttle, turn);

        // INTAKE CONTROL
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move(127);   // Spin full speed forward (intake in)
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            pros::lcd::print(3, "R2 pressed");
            intake_motor.move(-127);   // Spin full speed backward (intake out)
        } else {
            intake_motor.move(0);     // Stop intake when no button is held
        }

        // PNEUMATICS CONTROL -> A Toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
           piston_extended=!piston_extended; // toggle the piston state
           if(piston_extended==false) {
                clamp.extend(); // extend the piston
           }else{
                clamp.retract(); // retract the piston
           }
        }

        // FULL MOTORS CONTROL -> UP (L1) / DOWN (L2)
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            elevator_runu=!elevator_runu;
            elevator_rund=false;
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            elevator_rund=!elevator_rund;
            elevator_runu=false;
        }
        if(elevator_runu==true){
            full_motors.move(127);
        }else if(elevator_rund==true){
            full_motors.move(-127);
        }else{
            full_motors.brake();
        }
       
        // HALF MOTORS CONTROL -> UP (UP BUTTON) / DOWN (DOWN BUTTON)
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            armrunningu=!armrunningu;
            armrunningd=false;
        }
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            armrunningd=!armrunningd;
            armrunningu=false;
        }
        if(armrunningu==true){
            half_motors.move(127);
        }else if(armrunningd==true){
            half_motors.move(-127);
        }else{
            half_motors.brake();
        }
        // delay to save resources
        pros::delay(10);
    }
}