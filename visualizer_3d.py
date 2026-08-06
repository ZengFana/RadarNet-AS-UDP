"""
visualizer_3d.py
================

這是升級版的 2D/3D 混合視覺化模組。
包含功能：
1. 2D 雷達半圓形掃描視圖 (X-Y View) 與 3D 視圖 (3D View) 切換。
2. 支援設定感測器高度，讓 3D 點雲正確對齊地面 (Z=0)，並顯示雷達實體。
3. 支援目標軌跡 (Tracking Trajectories) 記錄與繪製。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence
import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QPainterPath, QColor, QBrush, QPen, QVector3D
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget, QStackedWidget, QGraphicsPathItem

try:
    import numpy as np
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    HAS_PYQTGRAPH = True
except Exception:
    np = None
    pg = None
    gl = None
    HAS_PYQTGRAPH = False


# ==========================================================
# 1. 顯示樣式集中管理
# ==========================================================
@dataclass
class ViewerStyle:
    background: str = "#000000"
    
    # 點 / 目標的顏色 (保持 RGBA 格式 0.0~1.0)
    dynamic_color: tuple = (0.33, 0.78, 0.92, 1.0) # 淺藍色
    static_color: tuple = (1.0, 0.0, 1.0, 1.0)     # 紫紅色
    target_color: tuple = (1.0, 1.0, 1.0, 1.0)     # 白色 (目標與軌跡)


# ==========================================================
# 2. 主要視覺化 Widget (支援 2D / 3D 切換)
# ==========================================================
class AreaScanner3DWidget(QWidget):

    def __init__(self, parent: Optional[QWidget] = None, style: Optional[ViewerStyle] = None) -> None:
        super().__init__(parent)
        self.style = style if style is not None else ViewerStyle()

        # --- 狀態變數 ---
        self._view_mode = "X-Y View"
        self._enable_zones = True
        self._critical_start_m = 0.0
        self._critical_end_m = 2.0
        self._warn_start_m = 2.0
        self._warn_end_m = 4.0
        self._projection_time_s = 2.0
        self._mounting_height_m = 2.0
        self._elevation_tilt_deg = 0.0
        self._yaw_offset_deg = 0.0
        self._x_offset_m = 0.0
        self._y_offset_m = 0.0
        self._enable_trajectory = True

        # --- 軌跡 (Trajectory) 記錄變數 ---
        self._target_history = {} # tid -> deque
        self._traj_2d = {}        # tid -> pg.PlotCurveItem
        self._traj_3d = {}        # tid -> gl.GLLinePlotItem
        self._box_3d = {}         # tid -> gl.GLLinePlotItem
        self._missing_frames = {} # tid -> target 消失後已保留的 frame 數
        self._target_smooth = {}  # tid -> 平滑後的 target 世界座標
        self._target_velocity = {} # tid -> smoothed frame-to-frame velocity
        self._max_history = 50    # 軌跡最多保留最近的 50 個點
        self._min_trail_step_m = 0.04
        self._max_trail_segment_m = 0.25
        self._prediction_frames = 8 # short occlusion hold before fading
        self._fade_frames = 24    # target 消失後，軌跡淡出的 frame 數
        self._smooth_alpha = 0.55 # 越大越貼近即時資料，越小越平滑
        self._max_target_jump_m = 2.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not HAS_PYQTGRAPH:
            placeholder = QLabel("無法匯入 pyqtgraph，請先確認安裝。")
            placeholder.setAlignment(Qt.AlignCenter)
            layout.addWidget(placeholder)
            self.stacked_widget = None
            return

        assert pg is not None
        pg.setConfigOptions(antialias=True)

        self.stacked_widget = QStackedWidget()
        layout.addWidget(self.stacked_widget)

        # ==================================================
        # A. 建立 2D 繪圖區 (X-Y View - 雷達半圓視圖)
        # ==================================================
        self.plot_2d = pg.PlotWidget(background=self.style.background)
        self.plot_2d.showGrid(x=True, y=True, alpha=0.3)
        self.plot_2d.setLabel('bottom', 'X [m]')
        self.plot_2d.setLabel('left', 'Y [m]')
        self.plot_2d.setAspectLocked(True) # 鎖定長寬比，確保半圓是正圓
        self.plot_2d.setYRange(0, 14)
        self.plot_2d.setXRange(-8, 8)

        # 1. Zone & 輔助線
        self.zone_warning_fill = QGraphicsPathItem()
        self.zone_warning_fill.setBrush(QBrush(QColor(210, 170, 0, 85)))
        self.zone_warning_fill.setPen(QPen(Qt.NoPen))
        self.plot_2d.addItem(self.zone_warning_fill)

        self.zone_critical = QGraphicsPathItem()
        self.zone_critical.setBrush(QBrush(QColor(139, 0, 0, 130)))
        self.zone_critical.setPen(QPen(Qt.NoPen))
        self.plot_2d.addItem(self.zone_critical)

        self.zone_warning = pg.PlotCurveItem(pen=pg.mkPen(color=(255, 255, 255, 200), width=2, style=Qt.DashLine))
        self.plot_2d.addItem(self.zone_warning)

        self.fov_line_left = pg.PlotCurveItem(pen=pg.mkPen(color=(200, 200, 200), width=1, style=Qt.DashLine))
        self.fov_line_right = pg.PlotCurveItem(pen=pg.mkPen(color=(200, 200, 200), width=1, style=Qt.DashLine))
        self.plot_2d.addItem(self.fov_line_left)
        self.plot_2d.addItem(self.fov_line_right)

        self.boresight_line = pg.PlotCurveItem([0, 0], [0, 20], pen=pg.mkPen('y', width=1))
        self.plot_2d.addItem(self.boresight_line)

        # 2. 點雲與目標圖層
        def to_pg_color(rgba: tuple) -> tuple:
            return tuple(int(c * 255) for c in rgba)

        self.scatter_2d_dynamic = pg.ScatterPlotItem(size=5, pen=None, brush=pg.mkBrush(*to_pg_color(self.style.dynamic_color)))
        self.scatter_2d_static = pg.ScatterPlotItem(size=7, pen=None, brush=pg.mkBrush(*to_pg_color(self.style.static_color)), symbol='s')
        self.scatter_2d_targets = pg.ScatterPlotItem(size=14, pen=pg.mkPen('w', width=2), brush=pg.mkBrush(0, 0, 0, 0), symbol='o')

        self.plot_2d.addItem(self.scatter_2d_dynamic)
        self.plot_2d.addItem(self.scatter_2d_static)
        self.plot_2d.addItem(self.scatter_2d_targets)

        self.stacked_widget.addWidget(self.plot_2d)

        # ==================================================
        # B. 建立 3D 繪圖區 (3D View)
        # ==================================================
        self.plot_3d = gl.GLViewWidget()
        self.plot_3d.setBackgroundColor(pg.mkColor(self.style.background))

        # 網格與座標軸
        grid = gl.GLGridItem()
        grid.setSize(x=20, y=20)
        grid.setSpacing(x=1, y=1)
        self.plot_3d.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(x=5, y=5, z=5)
        self.plot_3d.addItem(axis)

        self.zone_3d_critical = gl.GLMeshItem(color=(0.75, 0.05, 0.05, 0.28), smooth=False, drawEdges=False, glOptions='translucent')
        self.zone_3d_warning = gl.GLMeshItem(color=(1.0, 0.85, 0.15, 0.18), smooth=False, drawEdges=False, glOptions='translucent')
        self.plot_3d.addItem(self.zone_3d_warning)
        self.plot_3d.addItem(self.zone_3d_critical)

        # --- 雷達實體與安裝支柱 ---
        # 1. 安裝支柱 (灰色實線，代表安裝桿)
        self.mounting_pole = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,2]]), color=(0.7, 0.7, 0.7, 1.0), width=3, antialias=True)
        self.plot_3d.addItem(self.mounting_pole)

        # 2. 雷達實體方形 (純藍色實心方塊)
        verts = np.array([
            [-0.075, -0.075, -0.05], [0.075, -0.075, -0.05], [0.075, 0.075, -0.05], [-0.075, 0.075, -0.05],
            [-0.075, -0.075,  0.05], [0.075, -0.075,  0.05], [0.075, 0.075,  0.05], [-0.075, 0.075,  0.05]
        ])
        faces = np.array([
            [0,1,2], [0,2,3], [0,1,5], [0,5,4], [1,2,6], [1,6,5],
            [2,3,7], [2,7,6], [3,0,4], [3,4,7], [4,5,6], [4,6,7]
        ])
        md = gl.MeshData(vertexes=verts, faces=faces)
        self.sensor_body = gl.GLMeshItem(meshdata=md, smooth=False, color=(0.0, 0.3, 1.0, 1.0), glOptions='opaque')
        self.plot_3d.addItem(self.sensor_body)

        if hasattr(self, 'sensor_body'):
            self.sensor_body.resetTransform()

        # 3. 調整相機視角 (往下看，並聚焦在前方)
        self.plot_3d.opts['center'] = QVector3D(0, 3, 1)
        self.plot_3d.setCameraPosition(distance=15, elevation=30, azimuth=45)

        # --- 點雲與目標圖層 ---
        self.scatter_3d_dynamic = gl.GLScatterPlotItem(pos=np.empty((0, 3)), color=self.style.dynamic_color, size=5)
        self.scatter_3d_static = gl.GLScatterPlotItem(pos=np.empty((0, 3)), color=self.style.static_color, size=8)
        self.scatter_3d_targets = gl.GLScatterPlotItem(pos=np.empty((0, 3)), color=self.style.target_color, size=15)

        self.plot_3d.addItem(self.scatter_3d_dynamic)
        self.plot_3d.addItem(self.scatter_3d_static)
        self.plot_3d.addItem(self.scatter_3d_targets)

        self.stacked_widget.addWidget(self.plot_3d)

    # ------------------------------------------------------
    # 3. 對外公開：設定類方法
    # ------------------------------------------------------
    def set_view_mode(self, view_mode: str) -> None:
        """切換 2D / 3D 畫布"""
        if self.stacked_widget is None:
            return
        self._view_mode = view_mode.strip()
        if self._view_mode == "X-Y View":
            self.stacked_widget.setCurrentWidget(self.plot_2d)
        else:
            self.stacked_widget.setCurrentWidget(self.plot_3d)

    def set_zone_config(self, enable_zones: bool = True, critical_start_m: float = 0.0,
                        critical_end_m: float = 2.0, warn_start_m: float = 2.0,
                        warn_end_m: float = 4.0, projection_time_s: float = 2.0) -> None:
        """更新 2D 雷達視角的警示區與輔助線"""
        self._enable_zones = enable_zones
        self._critical_start_m = critical_start_m
        self._critical_end_m = critical_end_m
        self._warn_start_m = warn_start_m
        self._warn_end_m = warn_end_m

        if not HAS_PYQTGRAPH or self.stacked_widget is None:
            return

        warn_start, warn_end = self._display_warning_range()
        self.zone_critical.setVisible(enable_zones)
        self.zone_warning_fill.setVisible(enable_zones)
        self.zone_warning.setVisible(enable_zones)

        if enable_zones:
            # Critical Zone 使用 start/end 畫成半環，避免 GUI 設定的 Start 被忽略。
            path_crit = self._semicircle_band_path(self._critical_start_m, self._critical_end_m)
            self.zone_critical.setPath(path_crit)

            path_warn = self._semicircle_band_path(warn_start, warn_end)
            self.zone_warning_fill.setPath(path_warn)

            # 2D Warning 外緣再加一條乾淨虛線弧，保留官方警戒區讀法。
            warn_x, warn_y = self._semicircle_arc_points(warn_end)
            self.zone_warning.setData(warn_x, warn_y)

            self._update_3d_zone_meshes()
        else:
            self.zone_3d_critical.setVisible(False)
            self.zone_3d_warning.setVisible(False)

        # FOV 虛線 (設定為 ±60 度)
        fov_deg = 60
        line_length = max(15.0, self._warn_end_m * 2)
        x_left = -line_length * math.sin(math.radians(fov_deg))
        y_left = line_length * math.cos(math.radians(fov_deg))
        self.fov_line_left.setData([0, x_left], [0, y_left])

        x_right = line_length * math.sin(math.radians(fov_deg))
        y_right = line_length * math.cos(math.radians(fov_deg))
        self.fov_line_right.setData([0, x_right], [0, y_right])

    def _update_3d_zone_meshes(self) -> None:
        warn_start, warn_end = self._display_warning_range()
        self.zone_3d_critical.setVisible(True)
        self.zone_3d_warning.setVisible(True)
        self._set_3d_zone_mesh(self.zone_3d_critical, self._critical_start_m, self._critical_end_m)
        self._set_3d_zone_mesh(self.zone_3d_warning, warn_start, warn_end)

    def _display_warning_range(self) -> tuple:
        warn_start = self._warn_start_m
        warn_end = self._warn_end_m

        # 若 warning 跟 critical 完全重疊，仍保留官方式黃/紅雙區：黃區接在紅區外側。
        if warn_end <= self._critical_end_m:
            width = max(0.5, warn_end - warn_start)
            warn_start = self._critical_end_m
            warn_end = self._critical_end_m + width

        return warn_start, warn_end

    def _set_3d_zone_mesh(self, mesh_item, start_m: float, end_m: float) -> None:
        inner = max(0.0, min(start_m, end_m))
        outer = max(start_m, end_m)
        if outer <= 0:
            mesh_item.setVisible(False)
            return

        segments = 48
        vertices = []
        faces = []
        for i in range(segments + 1):
            angle = math.pi - (math.pi * i / segments)
            sin_a = math.sin(angle)
            cos_a = math.cos(angle)
            vertices.append([outer * cos_a, outer * sin_a, 0.01])
            vertices.append([inner * cos_a, inner * sin_a, 0.01])

        for i in range(segments):
            a = i * 2
            b = a + 1
            c = a + 2
            d = a + 3
            faces.append([a, c, b])
            faces.append([c, d, b])

        mesh_item.setMeshData(vertexes=np.array(vertices), faces=np.array(faces))

    def _semicircle_arc_points(self, radius_m: float) -> tuple:
        radius = max(0.0, radius_m)
        if radius <= 0:
            return [], []

        angles = np.linspace(math.pi, 0.0, 96)
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        return x, y

    def _semicircle_band_path(self, start_m: float, end_m: float) -> QPainterPath:
        inner = max(0.0, min(start_m, end_m))
        outer = max(start_m, end_m)

        path = QPainterPath()
        if outer <= 0:
            return path

        path.moveTo(outer, 0)
        path.arcTo(-outer, -outer, 2 * outer, 2 * outer, 0, -180)

        if inner > 0:
            path.lineTo(-inner, 0)
            path.arcTo(-inner, -inner, 2 * inner, 2 * inner, 180, 180)
        else:
            path.lineTo(0, 0)

        path.closeSubpath()
        return path

    def set_mount_config(self, mounting_height_m: float, elevation_tilt_deg: float,
                         yaw_offset_deg: float = 0.0, x_offset_m: float = 0.0,
                         y_offset_m: float = 0.0) -> None:
        """接收安裝高度，供 3D 視圖做 Z 軸補償，並連動實體方塊與支柱"""
        self._mounting_height_m = mounting_height_m
        self._elevation_tilt_deg = elevation_tilt_deg
        self._yaw_offset_deg = yaw_offset_deg
        self._x_offset_m = x_offset_m
        self._y_offset_m = y_offset_m

        if not HAS_PYQTGRAPH or self.stacked_widget is None:
            return

        h = self._mounting_height_m

        # 1. 更新支柱：起點為地面 (0,0,0)，終點為高度 H (0,0,h)
        if hasattr(self, 'mounting_pole'):
            self.mounting_pole.setData(pos=np.array([[0, 0, 0], [0, 0, h]]))

        # 2. 更新雷達方形實體位置
        if hasattr(self, 'sensor_body'):
            self.sensor_body.resetTransform()
            self.sensor_body.rotate(self._elevation_tilt_deg, 1, 0, 0)
            self.sensor_body.translate(0, 0, h)

    def set_smoothing_config(self, smoothing_alpha: float, max_target_jump_m: float) -> None:
        self._smooth_alpha = max(0.05, min(1.0, smoothing_alpha))
        self._max_target_jump_m = max(0.2, max_target_jump_m)

    def _apply_mount_transform(self, x: float, y: float, z: float) -> tuple:
        """套用安裝俯仰角與高度補償，回傳世界座標。"""
        tilt = math.radians(self._elevation_tilt_deg)
        cos_tilt = math.cos(tilt)
        sin_tilt = math.sin(tilt)
        y_tilted = y * cos_tilt - z * sin_tilt
        z_world = y * sin_tilt + z * cos_tilt + self._mounting_height_m

        yaw = math.radians(self._yaw_offset_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_world = x * cos_yaw - y_tilted * sin_yaw + self._x_offset_m
        y_world = x * sin_yaw + y_tilted * cos_yaw + self._y_offset_m
        return (x_world, y_world, z_world)

    def set_trajectory_enabled(self, enabled: bool) -> None:
        """[新增] 設定是否顯示軌跡"""
        self._enable_trajectory = enabled
        if not enabled:
            self.clear_trajectories()

    def clear_trajectories(self) -> None:
        """[新增] 只清除軌跡線而不影響點雲"""
        for item in self._traj_2d.values():
            self.plot_2d.removeItem(item)
        for item in self._traj_3d.values():
            self.plot_3d.removeItem(item)
        for item in self._box_3d.values():
            self.plot_3d.removeItem(item)
        self._target_history.clear()
        self._traj_2d.clear()
        self._traj_3d.clear()
        self._box_3d.clear()
        self._missing_frames.clear()
        self._target_smooth.clear()
        self._target_velocity.clear()

    def _trajectory_pen(self, alpha: float = 1.0):
        r, g, b, _ = self.style.target_color
        color = (int(r * 255), int(g * 255), int(b * 255), int(max(0.0, min(1.0, alpha)) * 255))
        return pg.mkPen(color=color, width=2, style=Qt.DashLine)

    def _trajectory_color_3d(self, alpha: float = 1.0) -> tuple:
        r, g, b, _ = self.style.target_color
        return (r, g, b, max(0.0, min(1.0, alpha)))

    def _set_trajectory_alpha(self, tid: int, alpha: float) -> None:
        if tid in self._traj_2d:
            self._traj_2d[tid].setPen(self._trajectory_pen(alpha))
        if tid in self._traj_3d:
            self._refresh_trajectory_items(tid, alpha)
        if tid in self._box_3d:
            position = self._target_smooth.get(tid)
            if position is not None:
                self._box_3d[tid].setData(pos=self._target_box_vertices(position), color=self._trajectory_color_3d(alpha))

    def _remove_trajectory(self, tid: int) -> None:
        if tid in self._traj_2d:
            self.plot_2d.removeItem(self._traj_2d[tid])
            del self._traj_2d[tid]
        if tid in self._traj_3d:
            self.plot_3d.removeItem(self._traj_3d[tid])
            del self._traj_3d[tid]
        if tid in self._box_3d:
            self.plot_3d.removeItem(self._box_3d[tid])
            del self._box_3d[tid]
        self._target_history.pop(tid, None)
        self._missing_frames.pop(tid, None)
        self._target_smooth.pop(tid, None)
        self._target_velocity.pop(tid, None)

    def _target_box_vertices(self, position: tuple, width: float = 0.7, depth: float = 0.7, height: float = 1.7):
        x, y, z = position
        z_bottom = max(0.02, z - height * 0.5)
        z_top = z_bottom + height
        x0, x1 = x - width * 0.5, x + width * 0.5
        y0, y1 = y - depth * 0.5, y + depth * 0.5

        corners = np.array([
            [x0, y0, z_bottom], [x1, y0, z_bottom], [x1, y1, z_bottom], [x0, y1, z_bottom],
            [x0, y0, z_top], [x1, y0, z_top], [x1, y1, z_top], [x0, y1, z_top],
        ])
        order = [0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4]
        return corners[order]

    def _smooth_target(self, target: dict) -> dict:
        tid = target["tid"]
        current = (target["x"], target["y"], target["z"])
        previous = self._target_smooth.get(tid)

        if previous is None:
            self._target_smooth[tid] = current
            self._target_velocity[tid] = (0.0, 0.0, 0.0)
            return target

        jump = math.dist(previous, current)
        if jump > self._max_target_jump_m:
            self._remove_trajectory(tid)
            self._target_smooth[tid] = current
            self._target_velocity[tid] = (0.0, 0.0, 0.0)
            return target

        alpha = self._smooth_alpha
        smoothed = tuple(previous[i] * (1.0 - alpha) + current[i] * alpha for i in range(3))
        self._target_smooth[tid] = smoothed
        self._target_velocity[tid] = tuple(smoothed[i] - previous[i] for i in range(3))
        return {**target, "x": smoothed[0], "y": smoothed[1], "z": smoothed[2]}

    def _append_trajectory_point(self, tid: int, point: tuple) -> None:
        history = self._target_history.get(tid)
        if history is None:
            return

        if not history:
            history.append(point)
            return

        previous = history[-1]
        distance = math.dist(previous, point)
        if distance < self._min_trail_step_m:
            history[-1] = point
            return

        if distance > self._max_trail_segment_m:
            steps = min(8, int(math.ceil(distance / self._max_trail_segment_m)))
            for step in range(1, steps):
                ratio = step / steps
                history.append(tuple(previous[i] + (point[i] - previous[i]) * ratio for i in range(3)))

        history.append(point)

    def _refresh_trajectory_items(self, tid: int, alpha: float = 1.0) -> None:
        pts = list(self._target_history.get(tid, []))
        if len(pts) > 1:
            if tid in self._traj_2d:
                self._traj_2d[tid].setData([p[0] for p in pts], [p[1] for p in pts])
                self._traj_2d[tid].setPen(self._trajectory_pen(alpha))
            if tid in self._traj_3d:
                self._traj_3d[tid].setData(pos=np.array(pts), color=self._trajectory_color_3d(alpha))

    def _predict_missing_target(self, tid: int) -> None:
        previous = self._target_smooth.get(tid)
        velocity = self._target_velocity.get(tid)
        if previous is None or velocity is None:
            return

        predicted = tuple(previous[i] + velocity[i] * 0.85 for i in range(3))
        self._target_smooth[tid] = predicted

        if tid in self._target_history:
            self._append_trajectory_point(tid, predicted)
            self._refresh_trajectory_items(tid)

        if tid in self._box_3d:
            self._box_3d[tid].setData(pos=self._target_box_vertices(predicted))

    # ------------------------------------------------------
    # 4. 對外公開：用 frame 更新畫面
    # ------------------------------------------------------
    def update_from_frame(self, frame, buffer_frame_count: int = 1) -> None:
        if not HAS_PYQTGRAPH or self.stacked_widget is None:
            return

        frame_info = self._normalize_frame(frame)
        dyn_pts_raw = frame_info["dynamic_points"]
        sta_pts_raw = frame_info["static_points"]
        targets_raw = frame_info["targets"]

        dyn_pts = [self._apply_mount_transform(p[0], p[1], p[2]) for p in dyn_pts_raw]
        sta_pts = [self._apply_mount_transform(p[0], p[1], p[2]) for p in sta_pts_raw]
        targets = []
        for target in targets_raw:
            x, y, z = self._apply_mount_transform(target["x"], target["y"], target["z"])
            targets.append(self._smooth_target({**target, "x": x, "y": y, "z": z}))
        
        is_2d = (self._view_mode == "X-Y View")

        # ==========================================
        # 軌跡處理 (Trajectory Tracking)
        # ==========================================
        if self._enable_trajectory:
            current_tids = set(t["tid"] for t in targets)
            # 目標短暫消失時不立刻刪線，先讓軌跡慢慢淡出，避免畫面看起來像顯示錯誤。
            for tid in list(self._target_history.keys()):
                if tid not in current_tids:
                    missing = self._missing_frames.get(tid, 0) + 1
                    self._missing_frames[tid] = missing
                    if missing >= self._fade_frames:
                        self._remove_trajectory(tid)
                    else:
                        if missing <= self._prediction_frames:
                            self._predict_missing_target(tid)
                            alpha = 1.0
                        else:
                            fade_span = max(1, self._fade_frames - self._prediction_frames)
                            alpha = 1.0 - ((missing - self._prediction_frames) / fade_span)
                        self._set_trajectory_alpha(tid, alpha)
        else:
            if self._target_history:
                self.clear_trajectories()

        # 2. 更新並畫出當前軌跡
        for t in targets:
            tid = t["tid"]
            if tid not in self._target_history:
                self._target_history[tid] = deque(maxlen=self._max_history)
                # 建立 2D 軌跡虛線
                self._traj_2d[tid] = pg.PlotCurveItem(pen=self._trajectory_pen(1.0))
                self.plot_2d.addItem(self._traj_2d[tid])
                # 建立 3D 軌跡實線
                self._traj_3d[tid] = gl.GLLinePlotItem(color=self._trajectory_color_3d(1.0), width=2, antialias=True)
                self.plot_3d.addItem(self._traj_3d[tid])
                # 建立 3D target 框，接近官方 People Counting 的框人效果

            self._missing_frames[tid] = 0
            self._set_trajectory_alpha(tid, 1.0)

            # 記錄軌跡點，使用已套用安裝高度與俯仰角補償後的座標。
            self._append_trajectory_point(tid, (t["x"], t["y"], t["z"]))

            # 刷新繪圖資料
            self._refresh_trajectory_items(tid)

        # ==========================================
        # 散佈點圖 (Scatter) 更新
        # ==========================================
        if is_2d:
            # 2D 視角 (忽略 Z 軸)
            self.scatter_2d_dynamic.setData(pos=[(p[0], p[1]) for p in dyn_pts] if dyn_pts else [])
            self.scatter_2d_static.setData(pos=[(p[0], p[1]) for p in sta_pts] if sta_pts else [])
            self.scatter_2d_targets.setData(pos=[(t["x"], t["y"]) for t in targets] if targets else [])
        else:
            # 3D 視角使用已套用安裝高度與俯仰角補償後的世界座標。
            self.scatter_3d_dynamic.setData(pos=np.array(dyn_pts) if dyn_pts else np.empty((0, 3)))
            self.scatter_3d_static.setData(pos=np.array(sta_pts) if sta_pts else np.empty((0, 3)))
            self.scatter_3d_targets.setData(pos=np.array([[t["x"], t["y"], t["z"]] for t in targets]) if targets else np.empty((0, 3)))

    def clear(self) -> None:
        """清空畫面，包含所有點雲與歷史軌跡"""
        if not HAS_PYQTGRAPH or self.stacked_widget is None:
            return
            
        self.scatter_2d_dynamic.setData(pos=[])
        self.scatter_2d_static.setData(pos=[])
        self.scatter_2d_targets.setData(pos=[])
        
        self.scatter_3d_dynamic.setData(pos=np.empty((0, 3)))
        self.scatter_3d_static.setData(pos=np.empty((0, 3)))
        self.scatter_3d_targets.setData(pos=np.empty((0, 3)))

        # 清除軌跡線
        for item in self._traj_2d.values():
            self.plot_2d.removeItem(item)
        for item in self._traj_3d.values():
            self.plot_3d.removeItem(item)
        for item in self._box_3d.values():
            self.plot_3d.removeItem(item)
            
        self._target_history.clear()
        self._traj_2d.clear()
        self._traj_3d.clear()
        self._box_3d.clear()
        self._missing_frames.clear()
        self._target_smooth.clear()
        self._target_velocity.clear()

    # ------------------------------------------------------
    # 5. 內部工具：frame 標準化
    # ------------------------------------------------------
    def _normalize_frame(self, frame) -> dict:
        if hasattr(frame, "header"):
            dynamic_points = [(float(p.x), float(p.y), float(p.z)) for p in getattr(frame, "dynamic_points", [])]
            static_points = [(float(p.x), float(p.y), float(p.z)) for p in getattr(frame, "static_points", [])]
            targets = [
                {
                    "tid": int(t.tid), "x": float(t.pos_x), "y": float(t.pos_y), "z": float(t.pos_z),
                    "vel_x": float(t.vel_x), "vel_y": float(t.vel_y), "vel_z": float(t.vel_z),
                }
                for t in getattr(frame, "targets", [])
            ]
            return {
                "frame_number": int(frame.header.frame_number),
                "dynamic_points": dynamic_points,
                "static_points": static_points,
                "targets": targets,
            }

        frame_dict = dict(frame)
        return {
            "frame_number": int(frame_dict.get("frame_number", 0)),
            "dynamic_points": list(frame_dict.get("dynamic_points", [])),
            "static_points": list(frame_dict.get("static_points", [])),
            "targets": list(frame_dict.get("tracked_targets", [])),
        }

# 相容別名
AreaScannerVisualizerWidget = AreaScanner3DWidget
__all__ = ['AreaScanner3DWidget', 'AreaScannerVisualizerWidget', 'ViewerStyle']
