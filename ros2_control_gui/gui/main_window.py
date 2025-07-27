"""
Main GUI window for the joint controller interface.
"""

import os
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSlider, 
    QGroupBox, QCheckBox, QDoubleSpinBox, QMessageBox, QFileDialog, QDialog
)
from PySide6.QtCore import QTimer, Qt

from .controllers_dialog import ControllersDialog


class MainWindow(QWidget):
    """Main GUI window"""
    
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Joint Controller GUI")
        self.ros_node = ros_node
        self.layout = QVBoxLayout(self)
        self.sliders = {}
        self.current_controller = None
        self.current_controller_info = None
        
        # Continuous command timer
        self.continuous_timer = QTimer()
        self.continuous_timer.timeout.connect(self.send_continuous_command)
        
        self._setup_ui()
        self._connect_signals()
        
        # Auto-request controllers on startup
        self.ros_node.request_controllers()
    
    def _setup_ui(self):
        """Setup the user interface"""
        # Controller selection section
        self._setup_controller_section()
        
        # Continuous command section
        self._setup_continuous_section()
        
        # Joint control section
        self._setup_joint_section()
        
        # Command buttons
        self._setup_command_buttons()
    
    def _setup_controller_section(self):
        """Setup controller selection section"""
        controller_group = QGroupBox("Controller Selection")
        controller_layout = QVBoxLayout()
        
        # YAML config file selection
        yaml_layout = QHBoxLayout()
        self.yaml_file_label = QLabel("No YAML config loaded")
        yaml_layout.addWidget(self.yaml_file_label)
        
        self.load_yaml_button = QPushButton("Load YAML Config")
        self.load_yaml_button.clicked.connect(self.load_yaml_config)
        yaml_layout.addWidget(self.load_yaml_button)
        
        controller_layout.addLayout(yaml_layout)
        
        self.controller_label = QLabel("No controller selected")
        controller_layout.addWidget(self.controller_label)
        
        self.select_button = QPushButton("Select Controller")
        self.select_button.clicked.connect(self.show_controller_dialog)
        controller_layout.addWidget(self.select_button)
        
        # Controller info display
        self.controller_info_label = QLabel("No controller info")
        controller_layout.addWidget(self.controller_info_label)
        
        controller_group.setLayout(controller_layout)
        self.layout.addWidget(controller_group)
    
    def _setup_continuous_section(self):
        """Setup continuous command section"""
        continuous_group = QGroupBox("Continuous Command Settings")
        continuous_layout = QHBoxLayout()
        
        # Continuous checkbox
        self.continuous_checkbox = QCheckBox("Send Continuous")
        self.continuous_checkbox.toggled.connect(self.on_continuous_toggled)
        continuous_layout.addWidget(self.continuous_checkbox)
        
        # Frequency setting
        freq_label = QLabel("Frequency (Hz):")
        continuous_layout.addWidget(freq_label)
        
        self.frequency_spinbox = QDoubleSpinBox()
        self.frequency_spinbox.setRange(0.1, 100.0)
        self.frequency_spinbox.setValue(10.0)
        self.frequency_spinbox.setSingleStep(0.5)
        self.frequency_spinbox.setDecimals(1)
        self.frequency_spinbox.valueChanged.connect(self.on_frequency_changed)
        continuous_layout.addWidget(self.frequency_spinbox)
        
        continuous_group.setLayout(continuous_layout)
        self.layout.addWidget(continuous_group)
    
    def _setup_joint_section(self):
        """Setup joint control section"""
        self.joint_group = QGroupBox("Joint Control")
        self.joint_layout = QVBoxLayout()
        self.joint_group.setLayout(self.joint_layout)
        self.joint_group.setEnabled(False)
        self.layout.addWidget(self.joint_group)
    
    def _setup_command_buttons(self):
        """Setup command buttons section"""
        command_buttons_layout = QHBoxLayout()
        
        self.command_button = QPushButton("Send Command")
        self.command_button.clicked.connect(self.send_command)
        self.command_button.setEnabled(False)
        command_buttons_layout.addWidget(self.command_button)
        
        self.reset_button = QPushButton("Reset All")
        self.reset_button.clicked.connect(self.reset_all_sliders)
        self.reset_button.setEnabled(False)
        self.reset_button.setStyleSheet("QPushButton { background-color: #f0f0f0; }")
        command_buttons_layout.addWidget(self.reset_button)
        
        self.layout.addLayout(command_buttons_layout)
    
    def _connect_signals(self):
        """Connect ROS-to-GUI signals"""
        self.ros_node.gui_signals.joint_state_updated.connect(
            self.update_joint_sliders
        )
        self.ros_node.gui_signals.controllers_received.connect(
            self.handle_controllers_received
        )
        self.ros_node.gui_signals.controller_selected.connect(
            self.handle_controller_selected
        )
        self.ros_node.gui_signals.controller_info_received.connect(
            self.handle_controller_info_received
        )
        self.ros_node.gui_signals.service_error.connect(
            self.handle_service_error
        )
    
    def load_yaml_config(self):
        """Load YAML configuration file through file dialog"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Controller YAML Configuration",
            "",
            "YAML files (*.yaml *.yml);;All files (*.*)"
        )
        
        if file_path:
            success = self.ros_node.load_yaml_config(file_path)
            if success:
                self.yaml_file_label.setText(f"Loaded: {os.path.basename(file_path)}")
                self.yaml_file_label.setToolTip(file_path)
                self.get_logger().info(f"Successfully loaded YAML config: {file_path}")
                
                # If a controller is already selected, refresh its info
                if self.current_controller:
                    self.ros_node.discover_controller_info(self.current_controller)
            else:
                QMessageBox.warning(self, "Error", f"Failed to load YAML file: {file_path}")
    
    def show_controller_dialog(self):
        """Show controller selection dialog"""
        self.select_button.setEnabled(False)
        self.select_button.setText("Loading...")
        
        success = self.ros_node.request_controllers()
        if not success:
            self.select_button.setEnabled(True)
            self.select_button.setText("Select Controller")
    
    def handle_controllers_received(self, controllers: list):
        """Handle successful controller list reception"""
        self.select_button.setEnabled(True)
        self.select_button.setText("Select Controller")
        
        if not controllers:
            self.controller_label.setText("No active controllers found")
            return
            
        dialog = ControllersDialog(controllers, self)
        if dialog.exec() == QDialog.Accepted and dialog.selected_controller:
            # Prompt for YAML config if not loaded
            if not self.ros_node.yaml_config.is_loaded():
                reply = QMessageBox.question(
                    self, 
                    "YAML Configuration", 
                    "No YAML configuration loaded. Would you like to load a controller configuration file?\n\nThis will ensure correct joint names and ordering.",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.Yes
                )
                if reply == QMessageBox.StandardButton.Yes:
                    self.load_yaml_config()
            
            self.ros_node.select_controller(dialog.selected_controller)
    
    def handle_service_error(self, error_message: str):
        """Handle service errors"""
        self.select_button.setEnabled(True)
        self.select_button.setText("Select Controller")
        QMessageBox.warning(self, "Service Error", error_message)
    
    def handle_controller_selected(self, controller_name: str):
        """Handle controller selection"""
        self.current_controller = controller_name
        self.controller_label.setText(f"Selected: {controller_name}")
    
    def handle_controller_info_received(self, controller_info: dict):
        """Handle controller configuration"""
        self.current_controller_info = controller_info
        
        # Display controller info
        controller_joint_names = controller_info.get('joint_names', [])
        config_source = controller_info.get('config_source', 'unknown')
        
        info_text = f"Name: {controller_info['name']}\n"
        info_text += f"Type: {controller_info['type']}\n"
        info_text += f"Controller Joints ({len(controller_joint_names)}): {controller_joint_names}\n"
        info_text += f"Config Source: {config_source}\n"
        info_text += f"Command Topics: {controller_info.get('command_topics', [])}"
        self.controller_info_label.setText(info_text)
        
        # Setup publisher
        self.ros_node.setup_publisher(controller_info)
        
        # Create joint sliders
        self.clear_sliders()
        for i, joint_name in enumerate(controller_joint_names):
            self.create_slider(joint_name, i)
        
        # Enable controls
        self.joint_group.setEnabled(True)
        self.command_button.setEnabled(True)
        self.reset_button.setEnabled(True)
    
    def create_slider(self, joint_name: str, index: int):
        """Create a slider for a joint"""
        row = QHBoxLayout()
        
        # Joint name label
        label = QLabel(f"{joint_name}:")
        label.setMinimumWidth(150)
        row.addWidget(label)
        
        # Command slider
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(-314, 314)  # -3.14 to 3.14 radians * 100
        slider.setValue(0)
        row.addWidget(slider)
        
        # Value display
        value_label = QLabel("0.000")
        value_label.setMinimumWidth(60)
        row.addWidget(value_label)
        
        # Connect slider to value update
        slider.valueChanged.connect(
            lambda v, lbl=value_label: lbl.setText(f"{v/100:.3f}")
        )
        
        # Connect mouse release event for auto-reset in continuous mode
        slider.sliderReleased.connect(
            lambda s=slider: self.on_slider_released(s)
        )
        
        self.joint_layout.addLayout(row)
        self.sliders[joint_name] = {'slider': slider, 'label': value_label, 'index': index}
    
    def clear_sliders(self):
        """Clear all existing sliders"""
        for widgets in self.sliders.values():
            widgets['slider'].deleteLater()
            widgets['label'].deleteLater()
        self.sliders.clear()
        
        # Clear layout
        while self.joint_layout.count():
            child = self.joint_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
    
    def on_continuous_toggled(self, checked: bool):
        """Handle continuous mode toggle"""
        if checked:
            frequency = self.frequency_spinbox.value()
            interval_ms = int(1000 / frequency)
            self.continuous_timer.start(interval_ms)
            self.command_button.setText("Stop Continuous")
            self.get_logger().info(f"Continuous mode started at {frequency} Hz")
        else:
            self.continuous_timer.stop()
            self.command_button.setText("Send Command")
            self.get_logger().info("Continuous mode stopped")
    
    def on_frequency_changed(self, frequency: float):
        """Handle frequency change"""
        if self.continuous_timer.isActive():
            interval_ms = int(1000 / frequency)
            self.continuous_timer.start(interval_ms)
            self.get_logger().info(f"Frequency changed to {frequency} Hz")
    
    def send_continuous_command(self):
        """Send command in continuous mode"""
        if self.current_controller and self.current_controller_info:
            self.send_command()
    
    def on_slider_released(self, slider: QSlider):
        """Handle slider release - reset to zero if in continuous mode"""
        if self.continuous_checkbox.isChecked():
            slider.setValue(0)
    
    def reset_all_sliders(self):
        """Reset all sliders to zero position"""
        for joint_name, widgets in self.sliders.items():
            widgets['slider'].setValue(0)
        
        self.get_logger().info("Reset all sliders to zero position")
        
        if self.current_controller and self.current_controller_info:
            self.send_command()
    
    def send_command(self):
        """Send command to controller"""
        if not self.current_controller or not self.current_controller_info:
            return
            
        # Get current slider values in correct order
        controller_joint_names = self.current_controller_info.get('joint_names', [])
        joint_positions = []
        
        for joint_name in controller_joint_names:
            if joint_name in self.sliders:
                position = self.sliders[joint_name]['slider'].value() / 100.0
                joint_positions.append(position)
            else:
                joint_positions.append(0.0)
                self.get_logger().warn(f"Joint {joint_name} not found in sliders, using 0.0")
        
        self.ros_node.send_joint_command(joint_positions)
    
    def update_joint_sliders(self, joint_data: dict):
        """Update slider display with current joint states"""
        # Optional: Update slider tooltips or background to show current position
        pass
    
    def get_logger(self):
        """Helper method to access ROS logger"""
        return self.ros_node.get_logger()
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.continuous_timer.isActive():
            self.continuous_timer.stop()
        event.accept()
