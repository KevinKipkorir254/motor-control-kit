#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscription_input_voltage_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10, std::bind(&MinimalSubscriber::update_input, this, std::placeholders::_1));
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalSubscriber::measure_joint_states, this, std::placeholders::_1));
    publisher_bj_model_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bj_model", 10);
    kalman_gain_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/kalman_gain", 10);
    p_variance_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/predictor_variance", 10);
    publisher_bj_model_prediction_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bj_model_prediction", 10);
    filtered_velocity_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filtered_velocity", 10);
    measured_velocity_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/measured_velocity", 10);

    //initialize kalman parameters
    this->declare_parameter("Q", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("R", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("P", rclcpp::PARAMETER_DOUBLE_ARRAY);

    try
    {
      {
        double Q = this->get_parameter("Q").as_double();
        double R = this->get_parameter("R").as_double();
        std::vector<double> P = this->get_parameter("P").as_double_array();

        tool_error_variance = R;
        q_model_noise = Q;
        predictor_variance[0] = P[0];
        predictor_variance[1] = P[1];
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    



    // Predict velocity values
    predict_velocity();

    // Predict predictor variance
    predict_predictor_variance();
  }

  void update_input(const std_msgs::msg::Float64MultiArray &msg)
  {
    input_voltage_ = msg.data[0];
  }

  void measure_joint_states(const sensor_msgs::msg::JointState &msg)
  {
    // get shaft position
    auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "shaft_joint");
    if (shaft_it != msg.name.end())
    {
      int shaft_index = std::distance(msg.name.begin(), shaft_it);
      shaft_position_ = msg.position[shaft_index];
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Shaft not found");
      return;
    }

    // get the preiod between last run and present run

    // calculate measured velocity
    measured_position[0] = shaft_position_;
    unfiltered_shaft_velocity_ = (measured_position[0] - measured_position[1]) / 0.01;
    measured_position[1] = measured_position[0];

    // publish measured velocity
    auto measured_velocity_message = std_msgs::msg::Float64MultiArray();
    measured_velocity_message.data.push_back(unfiltered_shaft_velocity_);
    measured_velocity_->publish(measured_velocity_message);

    // put your main code here, to run repeatedly:
    // OBTAINING THE DATA FEEDBACK
    // filtering the data
    shaft_velocity_ = 0.969 * yn_1 + 0.0155 * unfiltered_shaft_velocity_ + 0.0155 * xn_1; // low pass filter
    xn_1 = unfiltered_shaft_velocity_;
    yn_1 = shaft_velocity_;

    // publish filtered velocity
    auto filtered_velocity_message = std_msgs::msg::Float64MultiArray();
    filtered_velocity_message.data.push_back(shaft_velocity_);
    filtered_velocity_->publish(filtered_velocity_message);

    // Update kalman gain
    update_kalman_gains();

    // publish kalman gain
    auto kalman_gain_message = std_msgs::msg::Float64MultiArray();
    kalman_gain_message.data.push_back(Kalman_gain_);
    kalman_gain_->publish(kalman_gain_message);

    // publish predictor variance
    auto p_variance_message = std_msgs::msg::Float64MultiArray();
    p_variance_message.data.push_back(predictor_variance[0]);
    p_variance_->publish(p_variance_message);

    // update velocity
    update_velocity();
    update_outputs(); // so the model uses the kalman velocity estimate as the prevous velocity

    // publish update
    auto bj_model_message = std_msgs::msg::Float64MultiArray();
    bj_model_message.data.push_back(bj_model_velocity[1]);
    publisher_bj_model_->publish(bj_model_message);

    // update variances P_n_n
    update_predictor_variance();

    // Predict velocity values
    predict_velocity();

    //publish prediction
    auto prediction_message = std_msgs::msg::Float64MultiArray();
    prediction_message.data.push_back(bj_model_velocity[0]);
    publisher_bj_model_prediction_->publish(prediction_message);

    // Predict predictor variance
    predict_predictor_variance();

    // print anything
    RCLCPP_INFO(this->get_logger(), "Pos: '%f', Vel: '%f'", shaft_position_, shaft_velocity_);
  }

  void update_velocity()
  {
    bj_model_velocity[1] = bj_model_velocity[0] + Kalman_gain_ * (unfiltered_shaft_velocity_ - bj_model_velocity[0]);
  }

  void update_kalman_gains()
  {
    // Kalman gains
    Kalman_gain_ = predictor_variance[0] / (predictor_variance[0] + tool_error_variance);
  }

  void update_predictor_variance()
  {
    predictor_variance[1] = (1 - Kalman_gain_) * predictor_variance[0];

    // Update the predictor variance for the next iteration
    predictor_variance[0] = predictor_variance[1];
  }

  void predict_velocity()
  {
    update_inputs();
    output_velocity(&bj_model_);

    bj_model_velocity[0] = bj_model_;
  }

  void predict_predictor_variance()
  {
    predictor_variance[0] = predictor_variance[1] + q_model_noise;
  }

  void update_inputs()
  {
    past_inputs_bj.insert(past_inputs_bj.begin(), input_voltage_);

    past_inputs_bj.pop_back();
  }

  void update_outputs()
  {
    past_outputs_bj.insert(past_outputs_bj.begin(), bj_model_velocity[1]);

    past_outputs_bj.pop_back();
  }

  double BJ_Model(const std::vector<double> &B, const std::vector<double> &F, const std::vector<double> &C, const std::vector<double> &D,
                  const std::vector<double> &past_inputs, const std::vector<double> &past_outputs, const std::vector<double> &past_errors)
  {
    if (past_inputs.size() < B.size() || past_outputs.size() < F.size() - 1 || past_errors.size() < C.size() - 1)
    {
      throw std::invalid_argument("Insufficient past data for BJ model");
    }

    double output = 0.0;

    // Input contribution (B/F terms)
    double input_contribution = 0.0;
    for (size_t i = 0; i < B.size(); ++i)
    {
      input_contribution += B[i] * past_inputs[i];
    }
    for (size_t i = 1; i < F.size(); ++i)
    {
      input_contribution -= F[i] * past_outputs[i - 1];
    }

    // Noise contribution (C/D terms)
    double noise_contribution = 0.0;
    for (size_t i = 0; i < C.size(); ++i)
    {
      noise_contribution += C[i] * past_errors[i];
    }
    for (size_t i = 1; i < D.size(); ++i)
    {
      noise_contribution -= D[i] * past_errors[i - 1];
    }

    output = input_contribution / F[0] + noise_contribution / D[0];
    return output;
  }

  bool output_velocity(double *bj_model)
  {
    // Output the velocity

    *bj_model = BJ_Model(B_200_240_BJ, F_200_240_BJ, C_200_240_BJ, D_200_240_BJ, past_inputs_bj, past_outputs_bj, past_errors_bj);

    return true;
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_input_voltage_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_bj_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kalman_gain_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr p_variance_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_bj_model_prediction_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr filtered_velocity_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr measured_velocity_;

  double shaft_position_ = 0.0;
  double shaft_velocity_ = 0.0;
  double input_voltage_ = 0.0;

  double arx_model_ = 0.0;
  double armax_model_ = 0.0;
  double bj_model_ = 0.0;

  std::vector<double> arx_model_velocity = {0.0, 0.0};   /// v_n_n-1, v_n_n
  std::vector<double> armax_model_velocity = {0.0, 0.0}; /// v_n_n-1, v_n_n
  std::vector<double> bj_model_velocity = {0.0, 0.0};    /// v_n_n-1, v_n_n
  std::vector<double> measured_position = {0.0, 0.0};    /// v(k)   , v(k-1)
  std::vector<double> predictor_variance = {100.01, 0.0}; /// P_n_n-1, P_n_n

  // Low pass filter data
  double yn_1 = 0, xn_1 = 0;
  double tool_error_variance = 4.01;
  double q_model_noise = 0.001; // 4.0;
  double Kalman_gain_ = 0.0;   /// K_n_n-1, K_n_n
  double unfiltered_shaft_velocity_ = 0.0;

  /*----------------------------MODELS----------------------------------*/
  /// SYSTEM STATES
  // Example past inputs, outputs, and errors
  // Example past outputs and inputs

  // Example past inputs, outputs, and errors
  // Initialize past data
  std::vector<double> past_inputs_bj = {0.0, 0.0, 0.0};  // Max size required for BJ
  std::vector<double> past_outputs_bj = {0.0, 0.0, 0.0}; // Max size required for BJ
  std::vector<double> past_errors_bj = {0.0, 0.0, 0.0};  // Max size required for BJ

  // Define the Box-Jenkins model parameters
  std::vector<double> B_200_240_BJ = {0.0001875};           // B(z)
  std::vector<double> F_200_240_BJ = {1, -1.888, 0.8905};   // F(z)
  std::vector<double> C_200_240_BJ = {1, -0.0745, -0.3542}; // C(z)
  std::vector<double> D_200_240_BJ = {1, -0.9958};          // D(z)

  /*----------------------------MODELS----------------------------------*/
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}