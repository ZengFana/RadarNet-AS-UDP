"""
radar_client_gui.py
===================
接收端客戶端視窗：指定 Server IP 與 Port、定期發送心跳、相容轉碼並繪製於 visualizer_3d.py。
"""
import sys
import json
import socket
import time
from PySide6.QtCore import QThread, Signal, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGroupBox, QFormLayout, QLineEdit,
                             QSpinBox, QPushButton, QLabel, QComboBox, QCheckBox)
from visualizer_3d import AreaScanner3DWidget

class RadarClientWorker(QThread):
    frame_signal = Signal(dict)
    log_signal = Signal(str)

    def __init__(self, server_ip="127.0.0.1", server_port=9999, parent=None):
        super().__init__(parent)
        self.server_ip = server_ip
        self.server_port = server_port
        self._running = False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        server_addr = (self.server_ip, self.server_port)

        try:
            # 發送初次連線請求
            sock.sendto(b"CONNECT", server_addr)
            self.log_signal.emit(f"[Client] 已向 Server ({self.server_ip}:{self.server_port}) 發送連線請求...")

            last_ping_ts = 0

            while self._running:
                now = time.time()
                # 每 2 秒發送一次 Ping 心跳維持連線
                if now - last_ping_ts > 2.0:
                    sock.sendto(b"PING", server_addr)
                    last_ping_ts = now

                try:
                    data, _ = sock.recvfrom(65535)
                    if not data:
                        continue

                    frame_dict = json.loads(data.decode('utf-8'))
                    self.frame_signal.emit(frame_dict)

                except socket.timeout:
                    continue
                except Exception as e:
                    self.log_signal.emit(f"[Client 解析警告]: {e}")

        except Exception as exc:
            self.log_signal.emit(f"[Client Socket 錯誤]: {exc}")
        finally:
            self._running = False
            sock.close()
            self.log_signal.emit("[Client] 網路連線已關閉。")

class ClientMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RadarNet-AS 接收端 Visualizer")
        self.resize(1300, 850)
        self.worker = None

        central = QWidget(self)
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setFixedWidth(280)
        main_layout.addWidget(left_panel)

        box_net = QGroupBox("Server 連線設定")
        net_form = QFormLayout(box_net)
        self.edit_ip = QLineEdit("127.0.0.1")
        self.spin_port = QSpinBox()
        self.spin_port.setRange(1024, 65535)
        self.spin_port.setValue(9999)
        self.combo_view = QComboBox()
        self.combo_view.addItems(["3D View", "X-Y View"])

        net_form.addRow("Server IP 地址", self.edit_ip)
        net_form.addRow("Server Port 埠", self.spin_port)
        net_form.addRow("視圖模式", self.combo_view)
        left_layout.addWidget(box_net)

        box_display = QGroupBox("顯示選項")
        disp_form = QFormLayout(box_display)
        self.check_traj = QCheckBox("顯示即時追蹤軌跡")
        self.check_traj.setChecked(True)
        self.check_zones = QCheckBox("顯示紅黃警戒區")
        self.check_zones.setChecked(True)
        disp_form.addRow(self.check_traj)
        disp_form.addRow(self.check_zones)
        left_layout.addWidget(box_display)

        self.btn_start = QPushButton("連接伺服器 (Connect)")
        self.btn_stop = QPushButton("中斷連線 (Disconnect)")
        self.btn_stop.setEnabled(False)
        left_layout.addWidget(self.btn_start)
        left_layout.addWidget(self.btn_stop)

        self.lbl_status = QLabel("狀態: 閒置 (Idle)")
        left_layout.addWidget(self.lbl_status)
        left_layout.addStretch(1)

        # 右側：沿用原本視覺化畫布[span_8](start_span)[span_8](end_span)
        self.viewer = AreaScanner3DWidget()
        main_layout.addWidget(self.viewer, 1)

        self.viewer.set_view_mode("3D View")
        self.viewer.set_mount_config(mounting_height_m=2.0, elevation_tilt_deg=0.0)

        self.btn_start.clicked.connect(self.start_client)
        self.btn_stop.clicked.connect(self.stop_client)
        self.combo_view.currentTextChanged.connect(self.viewer.set_view_mode)
        self.check_traj.toggled.connect(self.viewer.set_trajectory_enabled)
        self.check_zones.toggled.connect(lambda en: self.viewer.set_zone_config(enable_zones=en))

    def start_client(self):
        self.viewer.clear()
        self.worker = RadarClientWorker(server_ip=self.edit_ip.text().strip(), server_port=self.spin_port.value())
        self.worker.frame_signal.connect(self.on_frame_received)
        self.worker.log_signal.connect(print)
        self.worker.finished.connect(self.on_stopped)
        self.worker.start()

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("狀態: 接收資料中...")

    def stop_client(self):
        if self.worker:
            self.worker.stop()

    def on_frame_received(self, frame_dict: dict):
        # 防呆相容物件封裝，防止 KeyError[span_9](start_span)[span_9](end_span)
        try:
            class DummyPoint:
                def __init__(self, d):
                    self.x = d.get("x", 0.0)
                    self.y = d.get("y", 0.0)
                    self.z = d.get("z", 0.0)

            class DummyTarget:
                def __init__(self, d):
                    self.tid = d.get("tid", 0)
                    self.pos_x = d.get("x", 0.0)
                    self.pos_y = d.get("y", 0.0)
                    self.pos_z = d.get("z", 0.0)
                    self.vel_x = d.get("vx", 0.0)
                    self.vel_y = d.get("vy", 0.0)
                    self.vel_z = d.get("vz", 0.0)

            class DummyHeader:
                def __init__(self, h):
                    self.frame_number = h.get("frame_number", 0)
                    self.num_tlvs = h.get("num_tlvs", 0)

            class DummyFrame:
                def __init__(self, fd):
                    self.header = DummyHeader(fd.get("header", {}))
                    self.dynamic_points = [DummyPoint(p) for p in fd.get("dynamic_points", [])]
                    self.static_points = [DummyPoint(p) for p in fd.get("static_points", [])]
                    self.targets = [DummyTarget(t) for t in fd.get("tracked_targets", [])]

            frame_obj = DummyFrame(frame_dict)
            self.viewer.update_from_frame(frame_obj, buffer_frame_count=1)

        except Exception as e:
            print(f"[Client GUI 畫面渲染錯誤]: {e}")

    def on_stopped(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("狀態: 已停止")

    def closeEvent(self, event):
        self.stop_client()
        if self.worker: self.worker.wait(2000)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ClientMainWindow()
    win.show()
    sys.exit(app.exec())
