from PyQt5.QtWidgets import QApplication
from eth_ctrl import GUI
import sys

if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = GUI()
    app.exec_()
