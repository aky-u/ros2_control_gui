"""
Dialog for controller selection.
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QPushButton
)
from PySide6.QtCore import Qt


class ControllersDialog(QDialog):
    """Dialog to show and select controllers"""
    
    def __init__(self, controllers: list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Active Controller")
        self.setModal(True)
        self.selected_controller = None
        self.resize(500, 300)
        
        layout = QVBoxLayout(self)
        
        # Info label
        info_label = QLabel("Select a controller to command:")
        layout.addWidget(info_label)
        
        # Controller list with improved display
        self.list_widget = QListWidget()
        self.list_widget.setAlternatingRowColors(True)
        
        for name, ctrl_type, state in controllers:
            # Create a more detailed display format
            display_text = f"{name}\n  Type: {ctrl_type}\n  State: {state}"
            item = QListWidgetItem(display_text)
            item.setData(Qt.UserRole, name)
            
            # Add tooltip with full information
            tooltip = f"Controller: {name}\nType: {ctrl_type}\nState: {state}"
            item.setToolTip(tooltip)
            
            self.list_widget.addItem(item)
        
        self.list_widget.itemDoubleClicked.connect(self.on_item_selected)
        layout.addWidget(self.list_widget)
        
        # Status label
        status_label = QLabel(f"Found {len(controllers)} active controller(s)")
        status_label.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(status_label)
        
        # Buttons
        button_layout = QHBoxLayout()
        select_button = QPushButton("Select")
        select_button.clicked.connect(self.on_select_clicked)
        select_button.setDefault(True)
        
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        
        button_layout.addWidget(select_button)
        button_layout.addWidget(cancel_button)
        layout.addLayout(button_layout)

    def on_item_selected(self, item):
        """Handle item double-click"""
        self.selected_controller = item.data(Qt.UserRole)
        self.accept()

    def on_select_clicked(self):
        """Handle select button click"""
        current_item = self.list_widget.currentItem()
        if current_item:
            self.selected_controller = current_item.data(Qt.UserRole)
            self.accept()
