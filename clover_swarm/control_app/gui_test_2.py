import sys
from PySide6 import QtWidgets, QtCore, QtUiTools

from qt_material import apply_stylesheet
from layout.main import Ui_MainWindow


def load_ui(filename, parent=None):
    ui_file = QtCore.QFile(filename)
    if not ui_file.open(QtCore.QIODevice.ReadOnly):
        return  # cannot load - ui_file.errorString()

    loader = QtUiTools.QUiLoader()
    ui = loader.load(ui_file, parent)
    ui_file.close()
    if not ui:
        return   # cannot load - loader.errorString()

    return ui

class MainWindow(Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setupUi(self)
        table = self.tableWidget
        table.setHorizontalHeaderLabels(["Имя коптера", "Состояние", "Что-то ещё"])



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    MainWindow = MainWindow()
    apply_stylesheet(app, theme='light_blue.xml')

    MainWindow.show()
    sys.exit(app.exec())
