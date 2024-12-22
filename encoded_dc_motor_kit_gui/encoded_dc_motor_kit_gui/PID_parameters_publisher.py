#!/usr/bin/env python3
"""
    Contains a GUI where the PID parameters (gains) and the set point are set. The parameters set are 
    - Proportional Gain KP,
    - Integral Gain KI
    - Derivative Gain KD
    - The Set Point SP

    The set parameters are published to the PID_parameters topic and are used to initialise the controller 
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QLineEdit, QPushButton
from PyQt5.QtCore import Qt

class GUIWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PID Controller')
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Program description
        description_label = QLabel("This program allows you to control the PID parameters.\n"
                                   "Adjust the sliders to set the Proportional, Integral, and Derivative gains,\n"
                                   "as well as the Set Point value. Press Enter to publish the values to ROS.")
        layout.addWidget(description_label)

        # Sliders section
        sliders_layout = QVBoxLayout()

        # Proportional Gain Slider
        proportional_layout = QHBoxLayout()
        self.proportional_label = QLabel('Proportional Gain:')
        self.proportional_value = QLineEdit()
        self.proportional_value.setReadOnly(True)
        self.proportional_slider = QSlider(Qt.Horizontal)
        self.proportional_slider.setMinimum(0)
        self.proportional_slider.setMaximum(100)
        self.proportional_slider.valueChanged.connect(lambda value: self.update_slider_value(value, self.proportional_value))
        proportional_layout.addWidget(self.proportional_label)
        proportional_layout.addWidget(self.proportional_slider)
        proportional_layout.addWidget(self.proportional_value)
        sliders_layout.addLayout(proportional_layout)

        # Integral Gain Slider
        integral_layout = QHBoxLayout()
        self.integral_label = QLabel('Integral Gain:')
        self.integral_value = QLineEdit()
        self.integral_value.setReadOnly(True)
        self.integral_slider = QSlider(Qt.Horizontal)
        self.integral_slider.setMinimum(0)
        self.integral_slider.setMaximum(100)
        self.integral_slider.valueChanged.connect(lambda value: self.update_slider_value(value, self.integral_value))
        integral_layout.addWidget(self.integral_label)
        integral_layout.addWidget(self.integral_slider)
        integral_layout.addWidget(self.integral_value)
        sliders_layout.addLayout(integral_layout)

        # Derivative Gain Slider
        derivative_layout = QHBoxLayout()
        self.derivative_label = QLabel('Derivative Gain:')
        self.derivative_value = QLineEdit()
        self.derivative_value.setReadOnly(True)
        self.derivative_slider = QSlider(Qt.Horizontal)
        self.derivative_slider.setMinimum(0)
        self.derivative_slider.setMaximum(100)
        self.derivative_slider.valueChanged.connect(lambda value: self.update_slider_value(value, self.derivative_value))
        derivative_layout.addWidget(self.derivative_label)
        derivative_layout.addWidget(self.derivative_slider)
        derivative_layout.addWidget(self.derivative_value)
        sliders_layout.addLayout(derivative_layout)

        # Set Point Slider
        set_point_layout = QHBoxLayout()
        self.set_point_label = QLabel('Set Point:')
        self.set_point_value = QLineEdit()
        self.set_point_value.setReadOnly(True)
        self.set_point_slider = QSlider(Qt.Horizontal)
        self.set_point_slider.setMinimum(0)
        self.set_point_slider.setMaximum(100)
        self.set_point_slider.valueChanged.connect(lambda value: self.update_slider_value(value, self.set_point_value))
        set_point_layout.addWidget(self.set_point_label)
        set_point_layout.addWidget(self.set_point_slider)
        set_point_layout.addWidget(self.set_point_value)
        sliders_layout.addLayout(set_point_layout)

        layout.addLayout(sliders_layout)

        # Enter button
        self.enter_button = QPushButton("Enter")
        self.enter_button.clicked.connect(self.publish_values)
        layout.addWidget(self.enter_button)

        self.setLayout(layout)

    def update_slider_value(self, value, line_edit):
        line_edit.setText(str(value / 100.0))

    def publish_values(self):
        proportional_value = float(self.proportional_value.text())
        integral_value = float(self.integral_value.text())
        derivative_value = float(self.derivative_value.text())
        set_point_value = float(self.set_point_value.text())

        self.node.publish_values(proportional_value, integral_value, derivative_value, set_point_value)

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.publisher_ = self.create_publisher(Float64, '/PID_parameters', 10)

    def publish_values(self, proportional, integral, derivative, set_point):
        msg = Float64()
        msg.data = proportional
        self.publisher_.publish(msg)

        msg.data = integral
        self.publisher_.publish(msg)

        msg.data = derivative
        self.publisher_.publish(msg)

        msg.data = set_point
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = PIDControllerNode()
    gui = GUIWidget()
    gui.node = node
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
