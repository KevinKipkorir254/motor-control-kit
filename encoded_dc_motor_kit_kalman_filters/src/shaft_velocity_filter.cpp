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
        subscription_input_voltage_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10, std::bind(&subscription_input_voltage_::update_input, this, std::placeholders::_1));
  }

  void update_input(const std_msgs::msg::Float64MultiArray &msg)
  {

  }

  void measure_shaft_velocity(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void update_velocity()
  {
  }

  void predict_velocity()
  {
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
  double BJ_Model(const std::vector<double> &B,
                  const std::vector<double> &F,
                  const std::vector<double> &C,
                  const std::vector<double> &D,
                  const std::vector<double> &past_inputs,
                  const std::vector<double> &past_outputs_F,
                  const std::vector<double> &past_errors,
                  const std::vector<double> &past_outputs_D)
  {
    // Validate sizes
    if (past_inputs.size() < B.size())
    {
      throw std::invalid_argument("Insufficient past_inputs for the given B array");
    }
    if (past_outputs_F.size() < F.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_outputs_F for the given F array");
    }
    if (past_errors.size() < C.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_errors for the given C array");
    }
    if (past_outputs_D.size() < D.size() - 1)
    {
      throw std::invalid_argument("Insufficient past_outputs_D for the given D array");
    }

    // Calculate B/F * u(t) term
    double term_BF = 0.0;

    // Contribution from inputs (B terms)
    for (size_t i = 0; i < B.size(); ++i)
    {
      term_BF += B[i] * past_inputs[i];
    }

    // Contribution from past outputs (F terms)
    for (size_t i = 1; i < F.size(); ++i)
    {
      term_BF -= F[i] * past_outputs_F[i - 1];
    }

    // Normalize by F[0]
    term_BF /= F[0];

    // Calculate C/D * e(t) term
    double term_CD = 0.0;

    // Contribution from errors (C terms)
    for (size_t i = 0; i < C.size(); ++i)
    {
      term_CD += C[i] * (i < past_errors.size() ? past_errors[i] : 0.0);
    }

    // Contribution from past outputs (D terms)
    for (size_t i = 1; i < D.size(); ++i)
    {
      term_CD -= D[i] * past_outputs_D[i - 1];
    }

    // Normalize by D[0]
    term_CD /= D[0];

    // Total output y(t)
    return term_BF + term_CD;
  }

  bool output_velocity(double *arx_model, double *armax_model, double *bj_model, double voltage_input)
  {
    // Output the velocity
    if (voltage_input < 40)
    {
      *armax_model = ARMAX_Model(A_0_40_ARMAX, B_0_40_ARMAX, C_0_40_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_0_40_ARX, B_0_40_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_0_40_BJ, F_0_40_BJ, C_0_40_BJ, D_0_40_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);
      return true;
    }
    else if ((voltage_input > 40) && (voltage_input < 80))
    {
      *armax_model = ARMAX_Model(A_40_80_ARMAX, B_40_80_ARMAX, C_40_80_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_40_80_ARX, B_40_80_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_40_80_BJ, F_40_80_BJ, C_40_80_BJ, D_40_80_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);
      return true;
    }
    else if ((voltage_input > 80) && (voltage_input < 120))
    {
      *armax_model = ARMAX_Model(A_80_120_ARMAX, B_80_120_ARMAX, C_80_120_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_80_120_ARX, B_80_120_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_80_120_BJ, F_80_120_BJ, C_80_120_BJ, D_80_120_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);
      return true;
    }
    else if ((voltage_input > 120) && (voltage_input < 160))
    {
      *armax_model = ARMAX_Model(A_120_160_ARMAX, B_120_160_ARMAX, C_120_160_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_120_160_ARX, B_120_160_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_120_160_BJ, F_120_160_BJ, C_120_160_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);
      return true;
    }
    else if ((voltage_input > 160) && (voltage_input < 200))
    {
      *armax_model = ARMAX_Model(A_160_200_ARMAX, B_160_200_ARMAX, C_160_200_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_160_200_ARX, B_160_200_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_160_200_BJ, F_160_200_BJ, C_160_200_BJ, D_160_200_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);

      return true;
    }
    else if ((voltage_input > 200) && (voltage_input < 240))
    {
      *armax_model = ARMAX_Model(A_200_240_ARMAX, B_200_240_ARMAX, C_200_240_ARMAX, past_outputs_armax, past_inputs_armax, past_errors_armax);
      *arx_model = ARX_Model(A_200_240_ARX, B_200_240_ARX, past_outputs, past_inputs);
      *bj_model = BJ_Model(B_200_240_BJ, F_200_240_BJ, C_200_240_BJ, D_200_240_BJ, past_inputs_bj, past_outputs_F_bj, past_errors_bj, past_outputs_D_bj);

      return true;
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_input_voltage_;

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
  std::vector<double> past_inputs_bj = {0.0};         // u(t-1)
  std::vector<double> past_outputs_F_bj = {0.0, 0.0}; // y_F(t-1), y_F(t-2)
  std::vector<double> past_errors_bj = {0.0, 0.0};    // e(t-1), e(t-2)
  std::vector<double> past_outputs_D_bj = {0.0};      // y_D(t-1)

  // Define the Box-Jenkins model parameters
  std::vector<double> B_0_40_BJ = {0.0};            // B(z)
  std::vector<double> F_0_40_BJ = {0.0, 0.0, 0.05}; // F(z)
  std::vector<double> C_0_40_BJ = {0.0, 0.0, 0.0};  // C(z)
  std::vector<double> D_0_40_BJ = {0.0, 0.0};       // D(z)

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