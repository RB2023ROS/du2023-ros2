#include "rclcpp/rclcpp.hpp"


class ParamExNode : public rclcpp::Node
{
private:
    ParamExNode() : Node("param_ex_node") {

        this->declare_parameter("string_param", "world");
        this->declare_parameter("int_param", 119);
        this->declare_parameter("float_param", 3.1415);
        this->declare_parameter("arr_param", std::vector<double>{1.0, 2.0, 3.0});
        this->declare_parameter("nested_param.string_param", "Wee Woo");

        rclcpp::Parameter str_param = this->get_parameter("string_param");
        rclcpp::Parameter int_param = this->get_parameter("int_param");
        rclcpp::Parameter float_param = this->get_parameter("float_param");
        rclcpp::Parameter arr_param = this->get_parameter("arr_param");
        rclcpp::Parameter nested_param = this->get_parameter("nested_param.string_param");
        
        std::string my_str = str_param.as_string();
        int my_int = int_param.as_int();
        int my_float = int_param.as_int();
        std::vector<double> my_double_array = double_array_param.as_double_array();
        
        RCLCPP_INFO(this->get_logger(), "str: %s, int: %s, double[]: %s",
                    str_param.value_to_string().c_str(),
                    int_param.value_to_string().c_str(),
                    double_array_param.value_to_string().c_str());
    }
};


// Code below is just to start the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParamExNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}