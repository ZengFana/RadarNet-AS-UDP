"""
serial_manager.py
=================

這個模組專門負責 Python 版 Area Scanner 的「序列埠通訊層」。

建議環境版本
------------
- Python 3.10.x
- pyserial == 3.5

設計目標
--------
1. 把 CLI Port / DATA Port 的開關與收發邏輯集中管理。
2. 讓 GUI 不需要直接碰 serial.Serial 細節，程式會比較乾淨。
3. 後續若要加上錄檔、錯誤重試、背景執行緒，也比較容易擴充。

這份檔案目前提供的核心能力
---------------------------
- 列出電腦目前可見的 COM Port
- 開啟 / 關閉 CLI Port 與 DATA Port
- 傳送單條 CLI 指令
- 傳送整份 cfg 檔
- 讀取 DATA Port 目前可用的二進位資料
- 清空序列埠緩衝區
- 做基本連線測試

重要觀念
--------
1. CLI Port
   - 用來送文字指令，例如 profileCfg、frameCfg、sensorStart。
   - 常見 baud rate 是 115200。

2. DATA Port
   - 用來收雷達輸出的二進位資料，也就是 TLV / packet。
   - 常見 baud rate 是 921600。

3. 兩個埠不要接反
   - 如果接反，常見現象會是：
     - CLI 指令沒有正常回覆
     - DATA 一直空的
     - 解析結果完全錯誤

4. 不能同時被多個程式占用
   - MATLAB GUI、TI Visualizer、Python 程式不能同時打開同一個 COM Port。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
import time

import serial
from serial import Serial
from serial.tools import list_ports


# ==========================================================
# 1. 設定資料結構
# ==========================================================
@dataclass(slots=True)
class SerialConfig:
    """
    集中保存序列埠設定。

    使用 dataclass 的好處：
    - 欄位集中，後續 GUI 很好同步
    - 比散落在各處的全域變數好維護
    - 日後若要存成 JSON / YAML 也方便
    """

    cli_port: str = "COM6"
    data_port: str = "COM5"
    cli_baud: int = 115200
    data_baud: int = 921600
    timeout_s: float = 1.0
    cfg_file: str = ""

    # 送 cfg 時，每條命令之間保留一點點時間，讓雷達有時間消化命令。
    command_delay_s: float = 0.02


@dataclass(slots=True)
class PortInfo:
    """
    把 pyserial 回傳的 port 資訊整理成比較好讀的格式。
    """

    device: str
    description: str
    hwid: str


# ==========================================================
# 2. 自訂錯誤類別
# ==========================================================
class SerialManagerError(Exception):
    """serial_manager 模組自己的基底錯誤。"""


class PortOpenError(SerialManagerError):
    """開啟 COM Port 失敗時使用。"""


class ConfigFileError(SerialManagerError):
    """cfg 檔案不存在或內容有問題時使用。"""


# ==========================================================
# 3. 核心類別：SerialManager
# ==========================================================
class SerialManager:
    """
    專門管理 Area Scanner 的 CLI / DATA 兩條序列埠。

    設計理念
    --------
    GUI 不應該自己直接 new serial.Serial，也不應該自己散落各種：
    - write()
    - readline()
    - in_waiting
    - reset_input_buffer()

    這些都應該集中在這個類別。這樣未來要：
    - 改 timeout
    - 增加 log
    - 增加錯誤處理
    - 增加 reconnect
    都會容易很多。
    """

    def __init__(self, config: SerialConfig) -> None:
        self.config = config

        # 兩個實際的 serial.Serial 物件，一開始尚未開啟，所以設成 None。
        self.cli_serial: Optional[Serial] = None
        self.data_serial: Optional[Serial] = None

    # ------------------------------------------------------
    # A. 靜態工具函式
    # ------------------------------------------------------
    @staticmethod
    def list_available_ports() -> List[PortInfo]:
        """
        列出電腦目前看到的所有序列埠。

        回傳結果示例：
        - COM5 / XDS110 Class Auxiliary Data Port
        - COM6 / XDS110 Class Application/User UART

        這個函式很適合接到 GUI 的「Refresh Ports」按鈕。
        """
        ports: List[PortInfo] = []
        for item in list_ports.comports():
            ports.append(
                PortInfo(
                    device=item.device,
                    description=item.description,
                    hwid=item.hwid,
                )
            )
        return ports

    # ------------------------------------------------------
    # B. 基本狀態檢查
    # ------------------------------------------------------
    def is_cli_open(self) -> bool:
        """檢查 CLI Port 是否已開啟。"""
        return self.cli_serial is not None and self.cli_serial.is_open

    def is_data_open(self) -> bool:
        """檢查 DATA Port 是否已開啟。"""
        return self.data_serial is not None and self.data_serial.is_open

    def is_fully_open(self) -> bool:
        """檢查 CLI / DATA 兩個埠是否都已正常開啟。"""
        return self.is_cli_open() and self.is_data_open()

    # ------------------------------------------------------
    # C. 開關序列埠
    # ------------------------------------------------------
    def open_ports(self) -> None:
        """
        依照目前設定開啟 CLI Port 與 DATA Port。

        注意
        ----
        若任一埠開啟失敗，會先把已開啟的另一個埠關掉，避免資源半開狀態。
        """
        self.close_ports()

        try:
            self.cli_serial = serial.Serial(
                port=self.config.cli_port,
                baudrate=self.config.cli_baud,
                timeout=self.config.timeout_s,
            )

            self.data_serial = serial.Serial(
                port=self.config.data_port,
                baudrate=self.config.data_baud,
                timeout=self.config.timeout_s,
            )
        except Exception as exc:
            self.close_ports()
            raise PortOpenError(
                f"無法開啟序列埠。CLI={self.config.cli_port}/{self.config.cli_baud}, "
                f"DATA={self.config.data_port}/{self.config.data_baud} | 原因：{exc}"
            ) from exc

    def close_ports(self) -> None:
        """
        關閉 CLI Port 與 DATA Port。

        這個函式採用「盡量關掉、不因單一錯誤中斷」的風格，
        避免某個 port 關閉失敗，反而讓另一個沒關到。
        """
        for ser in (self.cli_serial, self.data_serial):
            if ser is None:
                continue
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                # 關閉階段通常不需要因為單一錯誤整體中斷。
                pass

        self.cli_serial = None
        self.data_serial = None

    # ------------------------------------------------------
    # D. 緩衝區處理
    # ------------------------------------------------------
    def clear_buffers(self) -> None:
        """
        清空 CLI / DATA 的收發緩衝區。

        常見使用時機：
        - 剛開 port 後
        - 送 cfg 前
        - 測試連線前
        """
        if self.is_cli_open():
            assert self.cli_serial is not None
            self.cli_serial.reset_input_buffer()
            self.cli_serial.reset_output_buffer()

        if self.is_data_open():
            assert self.data_serial is not None
            self.data_serial.reset_input_buffer()
            self.data_serial.reset_output_buffer()

    # ------------------------------------------------------
    # E. CLI 指令相關
    # ------------------------------------------------------
    def send_cli_command(
        self,
        command: str,
        read_response: bool = True,
        response_wait_s: float = 0.05,
    ) -> str:
        """
        傳送單條 CLI 指令給雷達。

        參數
        ----
        command:
            不需要手動加換行，函式內部會自動補 '\n'。

        read_response:
            是否要等待並讀取 CLI 回應。

        response_wait_s:
            送出命令後先等待多久，再開始讀回應。
            有些板子或某些命令，如果等太短，可能會讀不到回應。

        回傳
        ----
        str
            CLI 回應文字。若沒讀回應，回傳空字串。
        """
        if not self.is_cli_open():
            raise PortOpenError("CLI Port 尚未開啟，無法送出命令。")

        assert self.cli_serial is not None

        clean_command = command.strip()
        payload = (clean_command + "\n").encode("utf-8")
        self.cli_serial.write(payload)
        self.cli_serial.flush()

        if not read_response:
            return ""

        # 稍等一下再讀，避免設備還沒把回應吐出來。
        time.sleep(response_wait_s)

        lines: List[str] = []
        deadline = time.time() + max(self.config.timeout_s, response_wait_s)

        while time.time() < deadline:
            waiting = self.cli_serial.in_waiting
            if waiting <= 0:
                time.sleep(0.01)
                continue

            raw = self.cli_serial.readline()
            if not raw:
                break

            text = raw.decode(errors="ignore").strip()
            if text:
                lines.append(text)

            # 若 CLI 回傳有 "Done" 或 "mmwDemo:/"，有些情況代表回應暫時結束。
            if text in {"Done", "mmwDemo:/"}:
                break

        return " | ".join(lines)

    def send_cfg_file(self, cfg_path: Optional[str] = None) -> List[str]:
        """
        把整份 cfg 檔逐行送進 CLI Port。

        回傳值
        ------
        List[str]
            每一行送出後的結果紀錄，方便 GUI 或 log 顯示。

        說明
        ----
        cfg 檔常見內容會包含：
        - sensorStop
        - flushCfg
        - profileCfg
        - chirpCfg
        - frameCfg
        - guiMonitor
        - sensorStart
        """
        path_str = cfg_path if cfg_path is not None else self.config.cfg_file
        path = Path(path_str)

        if not path.exists():
            raise ConfigFileError(f"找不到 cfg 檔：{path}")

        if not self.is_cli_open():
            raise PortOpenError("CLI Port 尚未開啟，無法送出 cfg。")

        logs: List[str] = []

        with path.open("r", encoding="utf-8", errors="ignore") as file:
            for raw_line in file:
                line = raw_line.strip()

                # 跳過註解與空白行。
                if not line or line.startswith("%"):
                    continue

                response = self.send_cli_command(
                    line,
                    read_response=True,
                    response_wait_s=self.config.command_delay_s,
                )

                log_text = f"傳送: {line} | 回傳: {response}"
                logs.append(log_text)

                # 某些命令之間留一點時間，比較不容易太快把設備塞爆。
                time.sleep(self.config.command_delay_s)

        return logs

    # ------------------------------------------------------
    # F. DATA Port 讀取
    # ------------------------------------------------------
    def read_data_once(self, max_bytes: Optional[int] = None) -> bytes:
        """
        從 DATA Port 讀取目前可用的資料。

        讀取策略
        --------
        - 預設會把 in_waiting 裡現有的資料全部讀掉。
        - 若 in_waiting == 0，會回傳空 bytes。
        - 若指定 max_bytes，則最多只讀那麼多。
        """
        if not self.is_data_open():
            raise PortOpenError("DATA Port 尚未開啟，無法讀取資料。")

        assert self.data_serial is not None

        waiting = self.data_serial.in_waiting
        if waiting <= 0:
            return b""

        read_size = waiting if max_bytes is None else min(waiting, max_bytes)
        return self.data_serial.read(read_size)

    # ------------------------------------------------------
    # G. 測試工具
    # ------------------------------------------------------
    def test_basic_connection(self) -> List[str]:
        """
        做一個「不碰 GUI、只測串口層」的基本連線檢查。

        這不是完整功能測試，但很適合先快速確認：
        1. COM Port 能不能開
        2. CLI / DATA 設定是不是明顯寫反

        回傳
        ----
        List[str]
            文字紀錄，方便直接印出或丟到 GUI log。
        """
        logs: List[str] = []
        logs.append(
            f"開始測試：CLI={self.config.cli_port}/{self.config.cli_baud}, "
            f"DATA={self.config.data_port}/{self.config.data_baud}"
        )

        self.open_ports()
        logs.append("CLI / DATA Port 開啟成功。")

        self.clear_buffers()
        logs.append("已清空緩衝區。")

        # version 是 mmWave 裝置常見的簡單 CLI 命令之一。
        # 有些韌體不一定支援，但仍可當作一個初步嘗試。
        try:
            response = self.send_cli_command("version", read_response=True, response_wait_s=0.1)
            logs.append(f"CLI version 回應：{response if response else '[空白]'}")
        except Exception as exc:
            logs.append(f"CLI version 指令測試失敗：{exc}")

        # DATA port 先觀察目前有沒有資料，不強迫一定要有。
        data = self.read_data_once()
        logs.append(f"DATA Port 目前收到 bytes 數：{len(data)}")

        return logs


# ==========================================================
# 4. 簡單自我測試區
# ==========================================================
if __name__ == "__main__":
    print("可見的序列埠如下：")
    for info in SerialManager.list_available_ports():
        print(f"- {info.device:>6} | {info.description} | {info.hwid}")
