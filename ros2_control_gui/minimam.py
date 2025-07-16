# -*- encoding: shift-jis -*-
import PySide6
from PySide6.QtWidgets import (QApplication,
                               QPushButton,     # ボタンを使うのに必要
                               QWidget)
import os
import sys


# PySide6のアプリ本体（ユーザがコーディングしていく部分）
class MainWindow(QWidget):
    def __init__(self, parent=None):
        # 親クラスの初期化
        super().__init__(parent)
        
        # ウィンドウタイトル
        self.setWindowTitle("PySide6で作ったアプリです。")
        
        # ボタンを表示するメソッド
        self.SetButton()
        
    # ボタンは別のメソッドに分けました
    def SetButton(self):
        # ボタンを使うことを宣言
        button = QPushButton(self)
        
        # ボタンに表示する文字
        button.setText("押してみよう！！")
        
        # ボタンを押したら実行させる処理
        # connectメソッド: 処理させるメソッド
        button.pressed.connect(self.CallbackButtonPressed)
        
        # ボタンを離したら実行させる処理（引数を指定する場合）
        # connectメソッド: 処理させるメソッド
        button.released.connect(lambda: self.CallbackButtonReleased(90))
        
    # ボタンが押されたら実行させるメソッド
    # connectメソッドから呼び出される
    def CallbackButtonPressed(self):
        print("akiyoshi!")
        
    # ボタンが離されたら実行させるメソッド（引数あり）
    # connectメソッドから呼び出される
    def CallbackButtonReleased(self, radian):
        print("横向くんだよ" + str(radian) + "度！")


if __name__ == "__main__":
    # 環境変数にPySide6を登録
    dirname = os.path.dirname(PySide6.__file__)
    plugin_path = os.path.join(dirname, 'plugins', 'platforms')
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_path
    
    app = QApplication(sys.argv)    # PySide6の実行
    window = MainWindow()           # ユーザがコーディングしたクラス
    window.show()                   # PySide6のウィンドウを表示
    sys.exit(app.exec())            # PySide6の終了
