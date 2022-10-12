#include <roboteq_ros_driver/serial_com.h>

geometry_msgs::Twist wheel_twist;
geometry_msgs::Twist cmd_twist;

roboteq_ros_driver::WheelRPM robot_vel;
roboteq_ros_driver::WheelRPM robot_cmd;

// Channels
const std::uint8_t left_wheel = 0;
const std::uint8_t right_wheel = 1;

void velCallback(const geometry_msgs::Twist::ConstPtr &msg){
    // m/s to rpm conversion
    robot_cmd.left_wheel_rpm = 201.0374 * (((msg->linear.x * 2) - (msg->angular.z * wheel_seperation)) / 2); 
    robot_cmd.right_wheel_rpm = 201.0374 * (((msg->linear.x * 2) + (msg->angular.z * wheel_seperation)) / 2); 
}

bool RoboteqSerialDriver::init (){
    this->port = "/dev/ttyACM0";
    this->baudrate = 115200;

    connect(this->port, this->baudrate);
}

bool RoboteqSerialDriver::connect (std::string port, int baudrate){
    try{
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
    }

    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
        initDriver();
        initMotor();
        initBLDC();
    }
    else{
        return -1;
    } 

    ros::Rate loop_rate(10);

    run ();
}

void RoboteqSerialDriver::run(){
    
}

// Runtime commands
// Note: The param arg could be the wheel number or the IO bit to set or reset
void RoboteqSerialDriver::setCommand (std::string command_type, std::uint8_t param,  std::int32_t value){
    std::stringstream runtime_cmd;
    runtime_cmd << command_type << " " << param << " " << value << "\r";
    ser.write(runtime_cmd.str());
    ser.flush();
}
void RoboteqSerialDriver::setCommand (std::string command_type, std::uint8_t param){

}
void RoboteqSerialDriver::setCommand (std::string command_type){
    
}

// Runtime queries
// Note: 
std::int32_t RoboteqSerialDriver::getQuery (std::string query_type, std::uint8_t param,  std::int32_t value){

}
std::int32_t RoboteqSerialDriver::getQuery (std::string query_type, std::uint8_t param){
    std::stringstream query_cmd; 
    std::string response;
    query_cmd << query_type << " " << param << "\r";
    ser.write(query_cmd.str());
    ser.flush();
    response = ser.read(ser.available());
    // clean data
    std::uint8_t equal = response.find("=");
    std::uint8_t delim = response.find(":");
    std::string temp;
    temp = response.substr(equal + 1, delim - 2);
    robot_vel.right_wheel_rpm = std::stoi(temp);
    temp = response.substr(delim + 1);
    robot_vel.left_wheel_rpm = std::stoi(temp);
    
}
std::int32_t RoboteqSerialDriver::getQuery (std::string query_type){

}

// Motor configurations
// Note: 
void RoboteqSerialDriver::setConfig (std::string config_type, std::uint8_t param, std::uint8_t action, std::uint8_t motor){
    std::stringstream config_cmd; 
    std::uint8_t value = action + motor;
    config_cmd << config_type << " " << param << " " << value << "\r";
    ser.write(config_cmd.str());
    ser.flush();
}

void RoboteqSerialDriver::setConfig (std::string config_type, std::uint8_t param,  std::int32_t value){
    std::stringstream config_cmd; 
    config_cmd << config_type << " " << param << " " << value << "\r";
    ser.write(config_cmd.str());
    ser.flush();
}

void RoboteqSerialDriver::setConfig (std::string config_type, std::uint8_t param){
    std::stringstream config_cmd;
    config_cmd << config_type << " " << param << "\r";
    ser.write(config_cmd.str());
    ser.flush();
}

void RoboteqSerialDriver::setConfig (std::string config_type){
    std::stringstream config_cmd; 
    config_cmd << config_type << "\r";
    ser.write(config_cmd.str());
    ser.flush();
}

void RoboteqSerialDriver::initDriver(){
    // Syntax: ^CPRI    priority   mode         Description: Configure communication priority  
    setConfig("^CPRI", 1, 0);          
    // Syntax: ^ECHOF    channel                Description: Enable(0)/Disable(1) echo
    setConfig("^ECHOF", 1); 
    // Syntax: ^FLCL    channel   value         Description: Automatic fault clearance  
    // Syntax: ^RSBR    channel   value         Description: Set RS232/RS485 baudrate, 0 for serial & 5 for inverted RS232
    setConfig("^RSBR", 0); 
    // Syntax: ^RWD     channel   value         Description: Serial Data Watchdog, 0 Disble, value for timer in ms
    setConfig("^RWD", 0);  
}

void RoboteqSerialDriver::initMotor(){
    // Syntax: ^ALIM    channel   value         Description: set max current per channel        
    setConfig("^ALIM", left_wheel, 40);          
    setConfig("^ALIM", right_wheel, 40);
    // Syntax: ^BLFB    channel   value         Description: select the closed loop feedback sensor, for Hall sensor value 1
    setConfig("^BLFB", left_wheel, 1);                    
    setConfig("^BLFB", right_wheel, 1);
    // Syntax: ^MLX     channel   value         Description: Configure Molex for Hall sensor, value 0
    setConfig("^MLX", left_wheel, 0);
    setConfig("^MLX", right_wheel, 0);
    // Syntax: ^MDIR    channel   value         Description: Configure motor direction, value is 0 for non inverted & 1 for inverted
    setConfig("^MDIR", left_wheel, 0);
    setConfig("^MDIR", right_wheel, 0);
    // Syntax: ^MMOD    channel   value         Description: Configure operating mode, value is 1 for closed loop speed
    setConfig("^MMOD", left_wheel, 1);
    setConfig("^MMOD", right_wheel, 1);
    /*
    // Syntax: ^MNRPM   channel   value         Description: Configure min rpm
    setConfig("^MNRPM", left_wheel, 1);
    setConfig("^MNRPM", right_wheel, 1);
    // Syntax: ^MXRPM   channel   value         Description: Configure max rpm
    setConfig("^MXRPM", left_wheel, 1);
    setConfig("^MXRPM", right_wheel, 1);
    // Syntax: ^MNRPM   channel   value         Description: Configure mixed mode
    setConfig("^MXMOD", left_wheel, 1);
    setConfig("^MXMOD", right_wheel, 1);
    // Syntax: ^KPG     channel   value         Description: Configure proportional gain
    setConfig("^KPG", left_wheel, 1);
    setConfig("^KPG", right_wheel, 1);
    // Syntax: ^KIG     channel   value         Description: Configure integral gain
    setConfig("^KIG", left_wheel, 1);
    setConfig("^KIG", right_wheel, 1);
    // Syntax: ^KDG     channel   value         Description: Configure derevative gain
    setConfig("^KDG", left_wheel, 1);
    setConfig("^KDG", right_wheel, 1);
    */
}

void RoboteqSerialDriver::initBLDC(){
    // Syntax: ^BMOD    channel   value         Description: Brushless Switching Mode, 0 for Trapezoidal      
    setConfig("^BMOD", left_wheel, 0);          
    setConfig("^BMOD", right_wheel, 0);
    // Syntax: ^BPOL    channel   value         Description: Number of Pole Pairs
    setConfig("^BPOL", left_wheel, 1);                    
    setConfig("^BPOL", right_wheel, 1);
    // Syntax: ^HPO     channel   value         Description: Hall Sensor Position Type, value is 0 or 1
    setConfig("^HPO", left_wheel, 0);
    setConfig("^HPO", right_wheel, 0);
    // Syntax: ^HSM    channel   value         Description: Hall Sensor Map, value ranges 0 to 5
    setConfig("^HSM", left_wheel, 0);
    setConfig("^HSM", right_wheel, 0);
    /*
    // Syntax: ^KPF     channel   value         Description: Configure proportional gain, channel is 3 for motor1 & 4 for motor 2 
    setConfig("^KPF", cc, 1);
    setConfig("^KPF", cc, 1);
    // Syntax: ^KIF     channel   value         Description: Configure integral gain, channel is 3 for motor1 & 4 for motor 2
    setConfig("^KIF", cc, 1);
    setConfig("^KIF", cc, 1);
    // Syntax: ^KDF     channel   value         Description: Configure derevative gain, channel is 3 for motor1 & 4 for motor 2
    setConfig("^KDF", cc, 1);
    setConfig("^KDF", cc, 1);
    */
}

void RoboteqSerialDriver::initIO(){
    // TODO
}

void RoboteqSerialDriver::getRPM (int right_wheel_rpm, int left_wheel_rpm){
    std::string response;
    ser.write("!BS\r");
    ser.flush();
    response = ser.read(ser.available());
    // clean data
    std::uint8_t equal = response.find("=");
    std::uint8_t delim = response.find(":");
    std::string temp;
    temp = response.substr(equal + 1, delim - 2);
    wheel_twist.linear.x = std::stoi(temp);
    temp = response.substr(delim + 1);
    left_wheel_rpm = std::stoi(temp);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "roboteq_serial_driver");
    ros::NodeHandle node;
    ros::Subscriber cmd_vel;
    // Custom rosmsg provide the speed of both the left and right wheel in [RPM]
    ros::Publisher wheel_vel;
    wheel_vel = node.advertise<geometry_msgs::Twist>("/wheel_vel", 1000);
    cmd_vel =  node.subscribe("/cmd_vel", 1000, velCallback);

    RoboteqSerialDriver controller;
    controller.setCommand("!S", left_wheel, robot_cmd.left_wheel_rpm);
    controller.setCommand("!S", right_wheel, robot_cmd.right_wheel_rpm);
    
}