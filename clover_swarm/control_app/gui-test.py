import sys
from PySide6 import QtWidgets, QtCore, QtUiTools
from qt_material import apply_stylesheet


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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    MainWindow = load_ui(r"ui\main.ui")
    apply_stylesheet(app, theme='light_blue.xml')

    MainWindow.show()
    sys.exit(app.exec())
