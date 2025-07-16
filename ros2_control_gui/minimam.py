import PySide6
from PySide6.QtWidgets import (QApplication,
                               QPushButton,     # Required to use buttons
                               QWidget)
import os
import sys


# Main PySide6 application (user-defined part)
class MainWindow(QWidget):
    def __init__(self, parent=None):
        # Initialize parent class
        super().__init__(parent)
        
        # Window title
        self.setWindowTitle("This is an app made with PySide6.")
        
        # Display the button
        self.SetButton()
        
    # Separated button setup into its own method
    def SetButton(self):
        # Declare the button
        button = QPushButton(self)
        
        # Set text on the button
        button.setText("Try pressing me!!")
        
        # Action when the button is pressed
        # Use the connect method to bind to the handler
        button.pressed.connect(self.CallbackButtonPressed)
        
        # Action when the button is released (with argument)
        # Use lambda to pass arguments
        button.released.connect(lambda: self.CallbackButtonReleased(90))
        
    # Method executed when the button is pressed
    # Called via connect method
    def CallbackButtonPressed(self):
        print("akiyoshi!")
        
    # Method executed when the button is released (with argument)
    # Called via connect method
    def CallbackButtonReleased(self, radian):
        print("uchda!")


if __name__ == "__main__":
    # Set PySide6 plugin path in environment variable
    dirname = os.path.dirname(PySide6.__file__)
    plugin_path = os.path.join(dirname, 'plugins', 'platforms')
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path
    
    app = QApplication(sys.argv)    # Run PySide6
    window = MainWindow()           # Instantiate the user-defined class
    window.show()                   # Show the PySide6 window
    sys.exit(app.exec())            # Exit PySide6
