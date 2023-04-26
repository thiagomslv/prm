#include <iostream>
#include "prm.h"

int main(int argc, char ** argv){

    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PRM>());
    rclcpp::shutdown();

    //PRM prm;
    //prm.prm(10, 3);
    
    return 0;
}