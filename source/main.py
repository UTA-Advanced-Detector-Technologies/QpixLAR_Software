from PyQt5.QtWidgets import QApplication
from eth_ctrl import GUI
import sys

if __name__ == "__main__":

    print("running the main program")
    app = QApplication(sys.argv)
    print("application started.. intializing the GUI")
    window = GUI()
    sys.exit(app.exec_())
