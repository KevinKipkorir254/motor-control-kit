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
    publisher_armax_model_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/armax_model", 10);
    publisher_arx_model_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arx_model", 10);
    publisher_bj_model_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bj_model", 10);
    kalman_gain_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/kalman_gain", 10);
    p_variance_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/predictor_variance", 10);

    // Predict velocity values
    predict_velocity();

    // Predict predictor variance
    predict_predictor_variance();
  }

  void update_input(const std_msgs::msg::Float64MultiArray &msg)
  {
    input_voltage_ = msg.data[0];

    // print anything
    RCLCPP_INFO(this->get_logger(), "Input: '%f'", input_voltage_);
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
    shaft_velocity_ = (measured_position[0] - measured_position[1]) / 0.01;
    measured_position[1] = measured_position[0];

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

    // publish update
    auto armax_model_message = std_msgs::msg::Float64MultiArray();
    armax_model_message.data.push_back(armax_model_velocity[1]);
    publisher_armax_model_->publish(armax_model_message);

    auto arx_model_message = std_msgs::msg::Float64MultiArray();
    arx_model_message.data.push_back(arx_model_velocity[1]);
    publisher_arx_model_->publish(arx_model_message);

    auto bj_model_message = std_msgs::msg::Float64MultiArray();
    bj_model_message.data.push_back(bj_model_velocity[1]);
    publisher_bj_model_->publish(bj_model_message);

    // update variances P_n_n
    update_predictor_variance();

    // Predict velocity values
    predict_velocity();

    // Predict predictor variance
    predict_predictor_variance();

    // print anything
    RCLCPP_INFO(this->get_logger(), "Pos: '%f', Vel: '%f'", shaft_position_, shaft_velocity_);
  }

  void update_velocity()
  {
    arx_model_velocity[1] = arx_model_velocity[0] + Kalman_gain_ * (shaft_velocity_ - arx_model_velocity[0]);
    armax_model_velocity[1] = armax_model_velocity[0] + Kalman_gain_ * (shaft_velocity_ - armax_model_velocity[0]);
    bj_model_velocity[1] = bj_model_velocity[0] + Kalman_gain_ * (shaft_velocity_ - bj_model_velocity[0]);
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
    output_velocity(&arx_model_, &armax_model_, &bj_model_, input_voltage_);
    update_errors();
    update_outputs();

    /// x_1_0
    arx_model_velocity[0] = arx_model_;
    armax_model_velocity[0] = armax_model_;
    bj_model_velocity[0] = bj_model_;
  }

  void predict_predictor_variance()
  {
    predictor_variance[0] = predictor_variance[1] + q_model_noise;
  }

  void update_inputs()
  {
    past_inputs.insert(past_inputs.begin(), input_voltage_);
    past_inputs_bj.insert(past_inputs_bj.begin(), input_voltage_);
    past_inputs_armax.insert(past_inputs_armax.begin(), input_voltage_);

    past_inputs.pop_back();
    past_inputs_bj.pop_back();
    past_inputs_armax.pop_back();
  }

  void update_errors()
  {
    // past_errors_armax.insert(past_errors_armax.begin(), input_voltage_ - armax_model_);
    //past_errors_bj.insert(past_errors_bj.begin(), input_voltage_ - bj_model_);

    // past_errors_armax.pop_back();
    //past_errors_bj.pop_back();
  }

  void update_outputs()
  {
    past_outputs.insert(past_outputs.begin(), arx_model_);
    past_outputs_bj.insert(past_outputs_bj.begin(), bj_model_);
    past_outputs_armax.insert(past_outputs_armax.begin(), armax_model_);

    past_outputs.pop_back();
    past_outputs_bj.pop_back();
    past_outputs_armax.pop_back();
  }

  // ARX Model Function
  double ARX_Model(const std::vector<double> &A, const std::vector<double> &B, const std::vector<double> &past_outputs, const std::vector<double> &past_inputs, double e_t = 0.0)
  {
    // Validate sizes
    if (past_outputs.size() < A.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_outputs for the given A array");
    }
    if (past_inputs.size() < B.size())
    {
      throw std::invalid_argument("Insufficient past_inputs for the given B array");
    }

    // Calculate denominator (A part)
    double denom = A[0];
    double y_t = 0.0;

    for (size_t i = 1; i < A.size(); ++i)
    {
      y_t -= A[i] * past_outputs[i - 1];
    }

    // Calculate numerator (B part)
    for (size_t i = 0; i < B.size(); ++i)
    {
      y_t += B[i] * past_inputs[i];
    }

    // Add noise (e(t))
    y_t += e_t;

    // Normalize with denominator
    y_t /= denom;

    return y_t;
  }

  // ARMAX Model Function
  double ARMAX_Model(const std::vector<double> &A,
                     const std::vector<double> &B,
                     const std::vector<double> &C,
                     const std::vector<double> &past_outputs,
                     const std::vector<double> &past_inputs,
                     const std::vector<double> &past_errors)
  {
    // Validate sizes
    if (past_outputs.size() < A.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_outputs for the given A array");
    }
    if (past_inputs.size() < B.size())
    {
      throw std::invalid_argument("Insufficient past_inputs for the given B array");
    }
    if (past_errors.size() < C.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_errors for the given C array");
    }

    // Calculate denominator (A part)
    double denom = A[0];
    double y_t = 0.0;

    // Contribution from past outputs (A terms)
    for (size_t i = 1; i < A.size(); ++i)
    {
      y_t -= A[i] * past_outputs[i - 1];
    }

    // Contribution from inputs (B terms)
    for (size_t i = 0; i < B.size(); ++i)
    {
      y_t += B[i] * past_inputs[i];
    }

    // Contribution from errors (C terms)
    for (size_t i = 1; i < C.size(); ++i)
    {
      y_t += C[i] * past_errors[i - 1];
    }

    // Normalize with denominator
    y_t /= denom;

    return y_t;
  } // Box-Jenkins Model Function
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

  bool output_velocity(double *arx_model, double *armax_model, double *bj_model, double voltage_input)
  {
    // Output the velocity

    *armax_model = ARMAX_Model(A_200_240_ARMAX, B_200_240_ARMAX, C_200_240_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
    *arx_model = ARX_Model(A_200_240_ARX, B_200_240_ARX, past_outputs, past_inputs);
    *bj_model = BJ_Model(B_200_240_BJ, F_200_240_BJ, C_200_240_BJ, D_200_240_BJ, past_inputs_bj, past_outputs_bj, past_errors_bj);

    return true;
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_input_voltage_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_armax_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_arx_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_bj_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kalman_gain_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr p_variance_;

  double shaft_position_ = 0.0;
  double shaft_velocity_ = 2.0;
  double input_voltage_ = 0.0;

  double arx_model_ = 0.0;
  double armax_model_ = 0.0;
  double bj_model_ = 0.0;

  std::vector<double> arx_model_velocity = {0.0, 0.0};   /// v_n_n-1, v_n_n
  std::vector<double> armax_model_velocity = {0.0, 0.0}; /// v_n_n-1, v_n_n
  std::vector<double> bj_model_velocity = {0.0, 0.0};    /// v_n_n-1, v_n_n
  std::vector<double> measured_position = {0.0, 0.0};    /// v(k)   , v(k-1)

  std::vector<double> predictor_variance = {0.01, 0.0}; /// P_n_n-1, P_n_n
  double tool_error_variance = 0.01;
  double q_model_noise = 1.01;//4.0;
  double Kalman_gain_ = 0.0; /// K_n_n-1, K_n_n

  /*----------------------------MODELS----------------------------------*/
  /// SYSTEM STATES
  // Example past inputs, outputs, and errors
  // Example past outputs and inputs
  std::vector<double> past_outputs = {1.0, 0.5}; // y(t-1), y(t-2)
  std::vector<double> past_inputs = {2.0};

  // ARX Models
  // 0 -  40
  // Define the ARX model parameters
  std::vector<double> A_0_40_ARX = {0.0, 0.0, 0.0}; // A(z)
  std::vector<double> B_0_40_ARX = {0.0};           // B(z)

  // 40 -  80
  // Define the ARX model parameters
  std::vector<double> A_40_80_ARX = {0.0, 0.0, 0.0}; // A(z)
  std::vector<double> B_40_80_ARX = {0.0};           // B(z)

  // 80 -  120
  // Define the ARX model parameters
  std::vector<double> A_80_120_ARX = {1, -1.467, 0.4725}; // A(z)
  std::vector<double> B_80_120_ARX = {0.0001091};         // B(z)

  // 120 -  160
  // Define the ARX model parameters
  std::vector<double> A_120_160_ARX = {1, -1.437, 0.4414}; // A(z)
  std::vector<double> B_120_160_ARX = {0.0001165};         // B(z)

  // 160 -  200
  // Define the ARX model parameters
  std::vector<double> A_160_200_ARX = {1, -1.537, 0.5411}; // A(z)
  std::vector<double> B_160_200_ARX = {0.0001312};         // B(z)

  // 200 -  240
  // Define the ARX model parameters
  std::vector<double> A_200_240_ARX = {1, -1.669, 0.6727}; // A(z)
  std::vector<double> B_200_240_ARX = {0.0001703};         // B(z)

  /// SYSTEM STATES
  // Example past inputs, outputs, and errors
  // Example past outputs and inputs

  // Example past outputs, inputs, and errors
  std::vector<double> past_outputs_armax = {1.0, 0.5}; // y(t-1), y(t-2)
  std::vector<double> past_inputs_armax = {2.0};       // u(t-1)
  std::vector<double> past_errors_armax = {0.1, -0.2}; // e(t-1), e(t-2)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_0_40_ARMAX = {0.0, 0.0, 0.0}; // A(z)
  std::vector<double> B_0_40_ARMAX = {0.0};           // B(z)
  std::vector<double> C_0_40_ARMAX = {0.0, 0.0};      // C(z)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_40_80_ARMAX = {0.0, 0.0, 0.0}; // A(z)
  std::vector<double> B_40_80_ARMAX = {0.0};           // B(z)
  std::vector<double> C_40_80_ARMAX = {0.0, 0.0};      // C(z)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_80_120_ARMAX = {1, -1.888, 0.8899}; // A(z)
  std::vector<double> B_80_120_ARMAX = {0.0001099};         // B(z)
  std::vector<double> C_80_120_ARMAX = {1, -0.9227};        // C(z)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_120_160_ARMAX = {1, -1.878, 0.8806}; // A(z)
  std::vector<double> B_120_160_ARMAX = {0.000128};          // B(z)
  std::vector<double> C_120_160_ARMAX = {1, -0.9374};        // C(z)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_160_200_ARMAX = {1, -1.875, 0.8774}; // A(z)
  std::vector<double> B_160_200_ARMAX = {0.0001467};         // B(z)
  std::vector<double> C_160_200_ARMAX = {1, -0.9204};        // C(z)

  // ARMAX Models
  // Define the ARMAX model parameters
  std::vector<double> A_200_240_ARMAX = {1, -1.885, 0.8872}; // A(z)
  std::vector<double> B_200_240_ARMAX = {0.0001897};         // B(z)
  std::vector<double> C_200_240_ARMAX = {1, -0.9199};        // C(z)

  /// SYSTEM STATES
  // Example past inputs, outputs, and errors
  // Example past outputs and inputs

  // Example past inputs, outputs, and errors
  // Initialize past data
  std::vector<double> past_inputs_bj = {0.0, 0.0, 0.0};  // Max size required for BJ
  std::vector<double> past_outputs_bj = {0.0, 0.0, 0.0}; // Max size required for BJ
  std::vector<double> past_errors_bj = {0.0, 0.0, 0.0};  // Max size required for BJ

  // Define the Box-Jenkins model parameters
  std::vector<double> B_0_40_BJ = {0.0};           // B(z)
  std::vector<double> F_0_40_BJ = {0.0, 0.0, 0.0}; // F(z)
  std::vector<double> C_0_40_BJ = {0.0, 0.0, 0.0}; // C(z)
  std::vector<double> D_0_40_BJ = {0.0, 0.0};      // D(z)

  // Define the Box-Jenkins model parameters
  std::vector<double> B_40_80_BJ = {0.0};           // B(z)
  std::vector<double> F_40_80_BJ = {0.0, 0.0, 0.0}; // F(z)
  std::vector<double> C_40_80_BJ = {0.0, 0.0, 0.0}; // C(z)
  std::vector<double> D_40_80_BJ = {0.0, 0.0};      // D(z)

  // Define the Box-Jenkins model parameters
  std::vector<double> B_80_120_BJ = {0.0001073};            // B(z)
  std::vector<double> F_80_120_BJ = {1, -0.01901, -0.4173}; // F(z)
  std::vector<double> C_80_120_BJ = {1, -0.9966};           // C(z)
  std::vector<double> D_80_120_BJ = {1, -1.894, 0.8964};    // D(z)

  // Define the Box-Jenkins model parameters
  std::vector<double> B_120_160_BJ = {0.0001263};            // B(z)
  std::vector<double> F_120_160_BJ = {1, -0.08266, -0.3516}; // F(z)
  std::vector<double> C_120_160_BJ = {1, -0.9881};           // C(z)
  std::vector<double> D_120_160_BJ = {1, -1.879, 0.8814};    // D(z)

  // Define the Box-Jenkins model parameters
  std::vector<double> B_160_200_BJ = {0.0001454};           // B(z)
  std::vector<double> F_160_200_BJ = {1, 0.03016, -0.3663}; // F(z)
  std::vector<double> C_160_200_BJ = {1, -0.9913};          // C(z)
  std::vector<double> D_160_200_BJ = {1, -1.876, 0.8784};   // D(z)

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