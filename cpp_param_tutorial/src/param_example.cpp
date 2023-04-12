#include "rclcpp/rclcpp.hpp"


class ParamExNode : public rclcpp::Node
{
public:
    ParamExNode() : Node("param_ex_node") {

        this->declare_parameter("string_param", "world");
        this->declare_parameter("int_param", 119);
        this->declare_parameter("double_param", 3.1415);
        this->declare_parameter("arr_param", std::vector<double>{1.0, 2.0, 3.0});
        this->declare_parameter("nested_param.string_param", "Wee Woo");

        rclcpp::Parameter str_param = this->get_parameter("string_param");
        rclcpp::Parameter int_param = this->get_parameter("int_param");
        rclcpp::Parameter double_param = this->get_parameter("double_param");
        rclcpp::Parameter arr_param = this->get_parameter("arr_param");
        rclcpp::Parameter nested_param = this->get_parameter("nested_param.string_param");
        
        std::string my_str = str_param.as_string();
        int my_int = int_param.as_int();
        double my_double = double_param.as_double();
        std::vector<double> my_arr = arr_param.as_double_array();
        std::string my_nested_str = nested_param.as_string();
        
        /** simplified version of above */
        // auto my_str = this->get_parameter("string_param").as_string();
        // auto my_int = this->get_parameter("int_param").as_int();
        // auto my_double = this->get_parameter("double_param").as_double();
        // auto my_arr = this->get_parameter("arr_param").as_double_array();
        // auto my_nested_str = this->get_parameter("nested_param.string_param").as_string();

        RCLCPP_INFO(this->get_logger(), "\nstr: %s \nint: %d \ndouble: %f \narr: %s \nnested: %s",
                    my_str.c_str(), my_int, my_double,
                    arr_param.value_to_string().c_str(),
                    my_nested_str.c_str());
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