"""
radar_server_gui.py
===================
發送端控制面板：管理雷達 COM Port、提供 Browse 按鈕選擇 CFG 檔、透過 UDP Socket 傳輸 JSON 資料。
"""
import sys
import json
import socket
import time
from pathlib import Path
from PySide6.QtCore import QThread, Signal, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QFormLayout, QComboBox, 
                             QSpinBox, QLineEdit, QPushButton, QPlainTextEdit, QLabel, QFileDialog)
from serial_manager import SerialManager, SerialConfig
from parser_as import parse_packet, AreaScannerParser

class RadarServerWorker(QThread):
    log_signal = Signal(str)
    status_signal = Signal(str)

    def __init__(self, config: SerialConfig, ip="127.0.0.1", port=9999, parent=None):
        super().__init__(parent)
        self.config = config
        self.ip = ip
        self.port = port
        self._running = False

    def stop(self):
        self._running = False

    def run(self):
        manager = SerialManager(self.config)
        parser = AreaScannerParser()
        self._running = True

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target_addr = (self.ip, self.port)

        try:
            self.status_signal.emit("正在開啟序列埠...")
            self.log_signal.emit(f"[Server] 開啟 CLI={self.config.cli_port}, DATA={self.config.data_port}")
            manager.open_ports()
            manager.clear_buffers()
            
            self.status_signal.emit("正在傳送 CFG...")
            self.log_signal.emit("[Server] 傳送雷達 CFG 設定檔...")
            for line in manager.send_cfg_file():
                self.log_signal.emit(f"  {line}")

            self.status_signal.emit("資料 UDP 串流中 (Streaming)")
            self.log_signal.emit(f"[Server] 開始透過 UDP 發送至 {self.ip}:{self.port}...")

            while self._running:
                raw = manager.read_data_once(max_bytes=8192)
                if not raw:
                    self.msleep(3)
                    continue

                parser.append_data(raw)
                packets = parser.extract_packets()

                for packet in packets:
                    if not self._running:
                        break
                    try:
                        frame = parse_packet(packet)
                        frame_dict = frame.to_dict()
                        
                        json_bytes = json.dumps(frame_dict).encode('utf-8')
                        udp_socket.sendto(json_bytes, target_addr)
                    except Exception as exc:
                        self.log_signal.emit(f"[Server 解析警告]: {exc}")

        except Exception as exc:
            self.log_signal.emit(f"[Server 臨界錯誤]: {exc}")
        finally:
            self.status_signal.emit("已停止")
            try: manager.send_cli_command("sensorStop", read_response=False)
            except: pass
            try: manager.close_ports()
            except: pass
            udp_socket.close()
            self.log_signal.emit("[Server] 伺服器與序列埠已安全停止。")

class ServerMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RadarNet-AS 發送端控制面板 (UDP Mode)")
        self.resize(650, 520)
        self.worker = None

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        box_serial = QGroupBox("雷達硬體與網路參數設定")
        form = QFormLayout(box_serial)
        
        self.combo_cli = QComboBox()
        self.combo_data = QComboBox()
        for info in SerialManager.list_available_ports():
            self.combo_cli.addItem(info.device)
            self.combo_data.addItem(info.device)
        self.combo_cli.setCurrentText("COM6")
        self.combo_data.setCurrentText("COM5")

        self.spin_cli_b = QSpinBox()
        self.spin_cli_b.setRange(9600, 3000000)
        self.spin_cli_b.setValue(115200)
        self.spin_data_b = QSpinBox()
        self.spin_data_b.setRange(9600, 3000000)
        self.spin_data_b.setValue(921600)

        cfg_widget = QWidget()
        cfg_layout = QHBoxLayout(cfg_widget)
        cfg_layout.setContentsMargins(0, 0, 0, 0)
        
        self.edit_cfg = QLineEdit()
        self.edit_cfg.setPlaceholderText("點擊 Browse... 按鈕選取 .cfg 檔案")
        self.btn_browse = QPushButton("Browse...")
        
        cfg_layout.addWidget(self.edit_cfg)
        cfg_layout.addWidget(self.btn_browse)

        self.edit_ip = QLineEdit("127.0.0.1")
        self.edit_port = QSpinBox()
        self.edit_port.setRange(1024, 65535)
        self.edit_port.setValue(9999)

        form.addRow("CLI 控制埠 (COM)", self.combo_cli)
        form.addRow("DATA 資料埠 (COM)", self.combo_data)
        form.addRow("CLI 波特率 (Baud)", self.spin_cli_b)
        form.addRow("DATA 波特率 (Baud)", self.spin_data_b)
        form.addRow("CFG 設定檔路徑", cfg_widget)
        form.addRow("目標 IP 地址", self.edit_ip)
        form.addRow("UDP 傳輸 Port", self.edit_port)
        layout.addWidget(box_serial)

        h_btn = QHBoxLayout()
        self.btn_test = QPushButton("硬體連線測試")
        self.btn_start = QPushButton("啟動伺服器 (Start)")
        self.btn_stop = QPushButton("停止伺服器 (Stop)")
        self.btn_stop.setEnabled(False)
        h_btn.addWidget(self.btn_test)
        h_btn.addWidget(self.btn_start)
        h_btn.addWidget(self.btn_stop)
        layout.addLayout(h_btn)

        self.lbl_status = QLabel("狀態: 閒置 (Idle)")
        layout.addWidget(self.lbl_status)
        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        layout.addWidget(self.txt_log)

        # 訊號綁定
        self.btn_browse.clicked.connect(self.browse_cfg_file)
        self.btn_test.clicked.connect(self.test_conn)
        self.btn_start.clicked.connect(self.start_server)
        self.btn_stop.clicked.connect(self.stop_server)

    @Slot()
    def browse_cfg_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "選擇雷達 CFG 設定檔 (Select CFG File)",
            str(Path.home()),
            "毫米波雷達設定檔 (*.cfg);;所有檔案 (*.*)"
        )
        if file_path:
            self.edit_cfg.setText(file_path)
            self.txt_log.appendPlainText(f"[系統] 已載入設定檔：{file_path}")

    def test_conn(self):
        s_config = SerialConfig(
            cli_port=self.combo_cli.currentText(),
            data_port=self.combo_data.currentText(),
            cli_baud=self.spin_cli_b.value(),
            data_baud=self.spin_data_b.value()
        )
        sm = SerialManager(s_config)
        try:
            self.txt_log.appendPlainText("\n".join(sm.test_basic_connection()))
        except Exception as e:
            self.txt_log.appendPlainText(f"[連線測試失敗]: {e}")

    def start_server(self):
        cfg_path = self.edit_cfg.text().strip()
        if not cfg_path:
            self.txt_log.appendPlainText("[錯誤] 請先點擊 'Browse...' 選擇 .cfg 檔案！")
            return
            
        s_config = SerialConfig(
            cli_port=self.combo_cli.currentText(),
            data_port=self.combo_data.currentText(),
            cli_baud=self.spin_cli_b.value(),
            data_baud=self.spin_data_b.value(),
            cfg_file=cfg_path
        )
        self.worker = RadarServerWorker(s_config, ip=self.edit_ip.text().strip(), port=self.edit_port.value())
        self.worker.log_signal.connect(self.txt_log.appendPlainText)
        self.worker.status_signal.connect(lambda txt: self.lbl_status.setText(f"狀態: {txt}"))
        self.worker.finished.connect(self.on_worker_finished)
        self.worker.start()
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_server(self):
        if self.worker: self.worker.stop()

    def on_worker_finished(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("狀態: 已停止")

    def closeEvent(self, event):
        self.stop_server()
        if self.worker: self.worker.wait(2000)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ServerMainWindow()
    win.show()
    sys.exit(app.exec())