#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "Eigen/Dense"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ModelPredictiveControl : public rclcpp::Node
{
public:
    ModelPredictiveControl()
        : Node("LeadCompensator"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        filtered_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filtered_velocity", 10);
        // subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/velocity/commands", 10, std::bind(&ModelPredictiveControl::update_reference_velocity, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ModelPredictiveControl::update_shaft_state_and_control_value, this, std::placeholders::_1));

        declare_parameters();
        read_parameters();
        mpcgain(A_, B_, C_, Nc_, Np_, Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e);

        RCLCPP_INFO(this->get_logger(), "Phi_Phi.rows: '%d', Phi_Phi.cols: '%d", Phi_Phi.rows(), Phi_Phi.cols());
        RCLCPP_INFO(this->get_logger(), "Phi_R.rows: '%d', Phi_R.cols: '%d", Phi_R.rows(), Phi_R.cols());
        RCLCPP_INFO(this->get_logger(), "Phi_F.rows: '%d', Phi_F.cols: '%d", Phi_F.rows(), Phi_F.cols());

        // print Phi_Phi, Phi_F, Phi_R using the rclcpp logger and a for loop
        for (int i = 0; i < Phi_Phi.rows(); i++)
        {
            for (int j = 0; j < Phi_Phi.cols(); j++)
            {
                RCLCPP_INFO(this->get_logger(), "Phi_Phi[%d][%d]: %.4f", i, j, Phi_Phi(i, j));
            }
        }

        for (int i = 0; i < Phi_F.rows(); i++)
        {
            for (int j = 0; j < Phi_F.cols(); j++)
            {
                RCLCPP_INFO(this->get_logger(), "Phi_F[%d][%d]: %.4f", i, j, Phi_F(i, j));
            }
        }

        for (int i = 0; i < Phi_R.rows(); i++)
        {
            for (int j = 0; j < Phi_R.cols(); j++)
            {
                RCLCPP_INFO(this->get_logger(), "Phi_R[%d][%d]: %.4f", i, j, Phi_R(i, j));
            }
        }
    }

private:
    void timer_callback()
    {
        auto message = std::make_shared<std_msgs::msg::Float64MultiArray>();
        message->data.push_back(0.0);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%.4f'", 0.0);
        publisher_->publish(*message);
    }

    void update_shaft_state_and_control_value(const sensor_msgs::msg::JointState &msg)
    {
        // get shaft position
        auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "shaft_joint");
        if (shaft_it != msg.name.end())
        {
            int shaft_index = std::distance(msg.name.begin(), shaft_it);
            shaft_position_ = msg.position[shaft_index];
            shaft_velocity_ = msg.velocity[shaft_index];
            RCLCPP_INFO(this->get_logger(), "Pos: '%f', Vel: '%f'", shaft_position_, shaft_velocity_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Shaft not found");
            return;
        }
        // filter velocity data
        shaft_velocity_ = filter_voltage_velocity_value(shaft_velocity_);
        publish_filtered_velocity_value(shaft_velocity_);

        double shaft_acceleration_ = shaft_velocity_old_ - shaft_velocity_;

        // get control value
        double control_output = update_control_value(shaft_velocity_, shaft_acceleration_);

        // publish control value
        publish_control_value(control_output);
    }

    void publish_control_value(double control_output)
    {

        if(control_output > 255)
        {
            control_output = 255;
        }
        else if(control_output < -255)
        {
            control_output = -255;
        }
        auto control_message = std_msgs::msg::Float64MultiArray();
        control_message.data.push_back(control_output);
        publisher_->publish(control_message);
    }

    void publish_filtered_velocity_value(double filtered_velocity)
    {
        auto filtered_velocity_message = std_msgs::msg::Float64MultiArray();
        filtered_velocity_message.data.push_back(filtered_velocity);
        filtered_velocity_publisher_->publish(filtered_velocity_message);
    }

    double filter_voltage_velocity_value(double shaft_velocity)
    {
        // OBTAINING THE DATA FEEDBACK
        // filtering the data
        double yn = 0.969 * yn_1 + 0.0155 * shaft_velocity + 0.0155 * xn_1;
        xn_1 = shaft_velocity;
        yn_1 = yn;
        return yn;
    }

    double update_control_value(double shaft_velocity, double shaft_acceleration)
    {
        Eigen::Vector3d x_ref;
        // fill x_ref with velocity and acceleration values
        x_ref << shaft_velocity - shaft_velocity_old, shaft_acceleration - shaft_acceleration_old, shaft_velocity;

        Eigen::VectorXd ref_vector = Eigen::VectorXd::Constant(10, 1);
        auto delat_u = (Phi_Phi + r_w_ * Eigen::MatrixXd::Identity(Nc_, Nc_)).inverse() * ((Phi_R * ref_vector.transpose()).colwise() - (Phi_F * x_ref));
        initial_input_ += delat_u(0, 0);

        shaft_velocity_old = shaft_velocity;
        shaft_acceleration_old = shaft_acceleration;

        return initial_input_;
    }

    void mpcgain(const Eigen::MatrixXd &Ap, const Eigen::MatrixXd &Bp, const Eigen::MatrixXd &Cp, int Nc, int Np,
                 Eigen::MatrixXd &Phi_Phi, Eigen::MatrixXd &Phi_F, Eigen::MatrixXd &Phi_R, Eigen::MatrixXd &A_e, Eigen::MatrixXd &B_e, Eigen::MatrixXd &C_e)
    {
        // Get matrix dimensions
        int m1 = Cp.rows();
        int n1 = Ap.cols();
        int n_in = Bp.cols();

        // Augmented matrices
        A_e = Eigen::MatrixXd::Identity(n1 + m1, n1 + m1);
        A_e.block(0, 0, n1, n1) = Ap;
        A_e.block(n1, 0, m1, n1) = Cp * Ap;

        B_e = Eigen::MatrixXd::Zero(n1 + m1, n_in);
        B_e.block(0, 0, n1, n_in) = Bp;
        B_e.block(n1, 0, m1, n_in) = Cp * Bp;

        C_e = Eigen::MatrixXd::Zero(m1, n1 + m1);
        C_e.block(0, n1, m1, m1) = Eigen::MatrixXd::Identity(m1, m1);

        // Compute h and F matrices
        Eigen::MatrixXd h(Np, n1 + m1);
        Eigen::MatrixXd F(Np, n1 + m1);

        h.row(0) = C_e;
        F.row(0) = C_e * A_e;

        for (int kk = 1; kk < Np; ++kk)
        {
            h.row(kk) = h.row(kk - 1) * A_e;
            F.row(kk) = F.row(kk - 1) * A_e;
        }

        // Compute v and initialize Phi matrix
        Eigen::MatrixXd v = h * B_e;
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(Np, Nc);

        Phi.col(0) = v.col(0);
        for (int i = 1; i < Nc; ++i)
        {
            Phi.block(i, i, Np - i, 1) = v.block(0, 0, Np - i, 1);
        }

        // Define reference signal
        Eigen::VectorXd BarRs = Eigen::VectorXd::Ones(Np);

        // Compute final matrices
        Phi_Phi = Phi.transpose() * Phi;
        Phi_F = Phi.transpose() * F;
        Phi_R = Phi.transpose() * BarRs;
    }

    void declare_parameters()
    {
        this->declare_parameter("model_predictive_inputs.Ap", rclcpp::PARAMETER_DOUBLE_ARRAY); // Example default 2x2 identity matrix
        this->declare_parameter("model_predictive_inputs.Bp", rclcpp::PARAMETER_DOUBLE_ARRAY); // Default 2x1
        this->declare_parameter("model_predictive_inputs.Cp", rclcpp::PARAMETER_DOUBLE_ARRAY); // Default 1x2
        this->declare_parameter("model_predictive_inputs.Dp", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("model_predictive_inputs.Np", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("model_predictive_inputs.Nc", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("model_predictive_inputs.initial_input", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("model_predictive_inputs.initial_output", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("model_predictive_inputs.r_w", rclcpp::PARAMETER_DOUBLE);
    }

    void read_parameters()
    {
        std::vector<double> Ap;
        std::vector<double> Bp;
        std::vector<double> Cp;
        double Dp;

        try
        {

            Ap = this->get_parameter("model_predictive_inputs.Ap").as_double_array();
            Bp = this->get_parameter("model_predictive_inputs.Bp").as_double_array();
            Cp = this->get_parameter("model_predictive_inputs.Cp").as_double_array();
            Dp = this->get_parameter("model_predictive_inputs.Dp").as_double();
            Np_ = this->get_parameter("model_predictive_inputs.Np").as_double();
            Nc_ = this->get_parameter("model_predictive_inputs.Nc").as_double();
            initial_input_ = this->get_parameter("model_predictive_inputs.initial_input").as_double();
            initial_output_ = this->get_parameter("model_predictive_inputs.initial_output").as_double();
            r_w_ = this->get_parameter("model_predictive_inputs.r_w").as_double();
        }
        catch (const std::exception &e)
        {
            // start error with error in brackes
            std::cerr << e.what() << '\n';
        }

        // Dynamically determine matrix dimensions
        // print out Ap using rclcpp logger using for loop and %.4f
        for (int i = 0; i < Ap.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Ap[%d]: %.4f", i, Ap[i]);
        }
        int A_rows = static_cast<int>(sqrt(Ap.size()));
        int A_cols = A_rows; // Assuming square matrix for state matrix A
        if (A_rows * A_cols != Ap.size())
        {
            throw std::runtime_error("Ap size is not a perfect square, cannot form a square matrix.");
        }

        // print out Bp using rclcpp logger using for loop and %.4f
        for (int i = 0; i < Bp.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Bp[%d]: %.4f", i, Bp[i]);
        }
        int B_rows = A_rows;
        int B_cols = static_cast<int>(Bp.size()) / B_rows;
        if (B_rows * B_cols != Bp.size())
        {
            throw std::runtime_error("Bp size does not match the expected B matrix dimensions.");
        }

        // print out Cp using rclcpp logger using for loop and %.4f
        for (int i = 0; i < Cp.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Cp[%d]: %.4f", i, Cp[i]);
        }
        int C_rows = static_cast<int>(Cp.size()) / A_cols;
        int C_cols = A_cols;
        if (C_rows * C_cols != Cp.size())
        {
            throw std::runtime_error("Cp size does not match the expected C matrix dimensions.");
        }

        // Resize matrices based on the dynamic dimensions
        A_.resize(A_rows, A_cols);
        B_.resize(B_rows, B_cols);
        C_.resize(C_rows, C_cols);
        D_.resize(1, 1); // D is treated as a 1x1 matrix

        // Populate matrices
        for (int i = 0; i < A_rows; ++i)
        {
            for (int j = 0; j < A_cols; ++j)
            {
                A_(i, j) = Ap[i * A_cols + j];
            }
        }

        for (int i = 0; i < B_rows; ++i)
        {
            for (int j = 0; j < B_cols; ++j)
            {
                B_(i, j) = Bp[i * B_cols + j];
            }
        }

        for (int i = 0; i < C_rows; ++i)
        {
            for (int j = 0; j < C_cols; ++j)
            {
                C_(i, j) = Cp[i * C_cols + j];
            }
        }

        D_(0, 0) = Dp;

        // Print matrices to terminal
        std::cout << "Matrix A (" << A_rows << "x" << A_cols << "):" << std::endl;
        for (int i = 0; i < A_rows; ++i)
        {
            for (int j = 0; j < A_cols; ++j)
            {
                std::cout << A_(i, j) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Matrix B (" << B_rows << "x" << B_cols << "):" << std::endl;
        for (int i = 0; i < B_rows; ++i)
        {
            for (int j = 0; j < B_cols; ++j)
            {
                std::cout << B_(i, j) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Matrix C (" << C_rows << "x" << C_cols << "):" << std::endl;
        for (int i = 0; i < C_rows; ++i)
        {
            for (int j = 0; j < C_cols; ++j)
            {
                std::cout << C_(i, j) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Matrix D (1x1): " << D_(0, 0) << std::endl;
    }

    std::string matrix_to_string(const Eigen::MatrixXd &mat) const
    {
        std::ostringstream oss;
        oss << mat;
        return oss.str();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr filtered_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    volatile double reference_velocity;
    volatile double shaft_position_ = 0.0;
    volatile double shaft_velocity_ = 0.0;
    volatile double shaft_velocity_old = 0.0;  // in MPC
    volatile double shaft_velocity_old_ = 0.0; // global
    volatile double shaft_acceleration_old = 0.0;
    volatile double yn_1 = 0.0, xn_1 = 0.0;

    /*---------------------CONTROLLER INIT VALUES-------------------------*/
    Eigen::MatrixXd A_; // State matrix
    Eigen::MatrixXd B_; // Input matrix
    Eigen::MatrixXd C_; // Output matrix
    Eigen::MatrixXd D_; // Feedthrough matrix
    Eigen::MatrixXd Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e;
    double initial_input_, initial_output_;
    double Np_, Nc_;
    double r_w_;
    /*---------------------CONTROLLER INIT VALUES-------------------------*/

    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelPredictiveControl>());
    rclcpp::shutdown();
    return 0;
}