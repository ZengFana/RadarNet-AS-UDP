"""
radar_server_gui.py
===================
發送端主控台：自動抓取本機區域網路 IP、管理雷達 COM Port、提供 Browse 選擇 CFG[span_0](start_span)[span_0](end_span)[span_1](start_span)[span_1](end_span)，
並以 UDP 廣播資料給所有連線的 Client。
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

def get_local_ip() -> str:
    """
    自動獲取本機在區域網路 (LAN/Wi-Fi) 中的真實 IP 地址
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 嘗試連接外部位址以獲取本機主要出口的 IP (不會真的發送數據)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

class MultiClientUDPServerWorker(QThread):
    log_signal = Signal(str)
    status_signal = Signal(str)

    def __init__(self, config: SerialConfig, port=9999, parent=None):
        super().__init__(parent)
        self.config = config
        self.port = port
        self._running = False
        self.clients = {}  # 紀錄 Client 位址 -> 最後心跳時間

    def stop(self):
        self._running = False

    def run(self):
        manager = SerialManager(self.config)
        parser = AreaScannerParser()
        self._running = True

        # 綁定 "0.0.0.0" 代表監聽本機所有的網路介面 (包含實體網卡與 Wi-Fi)，允許外部電腦連入
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.bind(("0.0.0.0", self.port))
        udp_socket.settimeout(0.01)

        self.log_signal.emit(f"[Server] UDP 伺服器已啟動，監聽 Port {self.port} (允許外部 IP 連線)...")
        self.status_signal.emit("初始化雷達...")

        try:
            manager.open_ports()
            manager.clear_buffers()

            self.log_signal.emit("[Server] 傳送雷達 CFG 設定檔...")
            for line in manager.send_cfg_file():
                self.log_signal.emit(f"  {line}")

            self.status_signal.emit("雷達傳輸中 (等待 Client 連線)")

            while self._running:
                # 1. 檢查是否有 Client 發送 握手/心跳 訊號 (Ping/Connect)
                try:
                    data, addr = udp_socket.recvfrom(1024)
                    msg = data.decode('utf-8').strip()
                    if msg.startswith("CONNECT") or msg.startswith("PING"):
                        if addr not in self.clients:
                            self.log_signal.emit(f"[Server] 新增 Client 連線: {addr[0]}:{addr[1]}")
                        self.clients[addr] = time.time()  # 更新 Client 心跳時間
                except socket.timeout:
                    pass

                # 2. 清理超過 5 秒沒有心跳的超時 Client
                now = time.time()
                dead_clients = [addr for addr, last_ts in self.clients.items() if now - last_ts > 5.0]
                for addr in dead_clients:
                    del self.clients[addr]
                    self.log_signal.emit(f"[Server] Client 逾時中斷: {addr[0]}:{addr[1]}")

                # 3. 讀取雷達數據並發送給所有已註冊的 Client
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

                        # 廣播發送給所有在線上的 Client
                        for client_addr in list(self.clients.keys()):
                            udp_socket.sendto(json_bytes, client_addr)

                        if self.clients:
                            self.status_signal.emit(f"串流中 (線上 Client: {len(self.clients)} 個)")
                        else:
                            self.status_signal.emit("等待 Client 連線...")

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
        self.setWindowTitle("RadarNet-AS 發送端主控台 (Server)")
        self.resize(680, 560)
        self.worker = None

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        box_serial = QGroupBox("Server 網路與雷達硬體參數設定")
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

        # CFG 選擇 Browse... 按鈕[span_2](start_span)[span_2](end_span)
        cfg_widget = QWidget()
        cfg_layout = QHBoxLayout(cfg_widget)
        cfg_layout.setContentsMargins(0, 0, 0, 0)
        self.edit_cfg = QLineEdit()
        self.edit_cfg.setPlaceholderText("點擊 Browse... 選擇 .cfg 檔案")
        self.btn_browse = QPushButton("Browse...")
        cfg_layout.addWidget(self.edit_cfg)
        cfg_layout.addWidget(self.btn_browse)

        # 自動顯示本機區域網路 IP
        local_ip = get_local_ip()
        self.lbl_ip_info = QLabel(f"<b>{local_ip}</b> <font color='gray'>(讓其他 Client 輸入此 IP)</font>")

        self.edit_port = QSpinBox()
        self.edit_port.setRange(1024, 65535)
        self.edit_port.setValue(9999)

        form.addRow("CLI 控制埠 (COM)", self.combo_cli)
        form.addRow("DATA 資料埠 (COM)", self.combo_data)
        form.addRow("CLI 波特率 (Baud)", self.spin_cli_b)
        form.addRow("DATA 波特率 (Baud)", self.spin_data_b)
        form.addRow("CFG 設定檔路徑", cfg_widget)
        form.addRow("本機 Server IP", self.lbl_ip_info) # 自動取得實體 IP 顯示於此
        form.addRow("Server 監聽 Port", self.edit_port)
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
        self.worker = MultiClientUDPServerWorker(s_config, port=self.edit_port.value())
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
