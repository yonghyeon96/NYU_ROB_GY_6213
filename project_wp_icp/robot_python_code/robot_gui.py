import asyncio
import math
from matplotlib import pyplot as plt
import matplotlib
import numpy as np
from nicegui import ui, app, run
from time import time
from fastapi import Response

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

from robot import Robot, LOCALIZATION_MODE
import robot_python_code
import parameters

logging = False
stream_video = False


def _cv2_convert(frame: np.ndarray) -> bytes:
    _, buf = cv2.imencode('.jpg', frame)
    return buf.tobytes()

def get_time_in_ms():
    return int(time() * 1000)


@ui.page('/')
def main():

    robot = Robot()

    # ------------------------------------------------------------------ lidar
    max_lidar_range  = 12
    lidar_angle_res  = 2
    num_angles       = int(360 / lidar_angle_res)
    lidar_dist_list  = [max_lidar_range] * num_angles
    lidar_cos_list   = [math.cos(i * lidar_angle_res / 180 * math.pi) for i in range(num_angles)]
    lidar_sin_list   = [math.sin(i * lidar_angle_res / 180 * math.pi) for i in range(num_angles)]

    dark = ui.dark_mode()
    dark.value = True

    if stream_video and _CV2_AVAILABLE:
        video_capture = cv2.VideoCapture(parameters.camera_id)

        @app.get('/video/frame')
        async def grab_video_frame() -> Response:
            if not video_capture.isOpened():
                return Response(content=b'', media_type='image/jpeg')
            _, frame = await run.io_bound(video_capture.read)
            if frame is None:
                return Response(content=b'', media_type='image/jpeg')
            jpeg = await run.cpu_bound(_cv2_convert, frame)
            return Response(content=jpeg, media_type='image/jpeg')

    # ------------------------------------------------------------------ lidar update
    def update_lidar_data():
        for i in range(robot.robot_sensor_signal.num_lidar_rays):
            dist_mm = robot.robot_sensor_signal.distances[i]
            angle   = 360 - robot.robot_sensor_signal.angles[i]
            if dist_mm > 20 and abs(angle) < 360:
                idx = max(0, min(num_angles - 1,
                                 int((angle - lidar_angle_res / 2) / lidar_angle_res)))
                lidar_dist_list[idx] = dist_mm / 1000

    # ------------------------------------------------------------------ commands
    def update_commands():
        if robot.running_trial:
            dt = get_time_in_ms() - robot.trial_start_time
            if dt > parameters.trial_time:
                robot.running_trial = False
                speed_switch.value   = False
                steering_switch.value = False
                robot.extra_logging  = True
        if getattr(robot, 'extra_logging', False):
            dt = get_time_in_ms() - robot.trial_start_time
            if dt > parameters.trial_time + parameters.extra_trial_log_time:
                logging_switch.value  = False
                robot.extra_logging   = False

        cmd_speed = slider_speed.value if speed_switch.value else 0
        cmd_steer = slider_steering.value if steering_switch.value else 0
        return cmd_speed, cmd_steer

    def update_connection():
        if udp_switch.value:
            if not robot.connected_to_hardware:
                udp, ok = robot_python_code.create_udp_communication(
                    parameters.arduinoIP, parameters.localIP,
                    parameters.arduinoPort, parameters.localPort,
                    parameters.bufferSize)
                if ok:
                    robot.setup_udp_connection(udp)
                    robot.connected_to_hardware = True
                else:
                    udp_switch.value = False
                    robot.connected_to_hardware = False
        else:
            if robot.connected_to_hardware:
                robot.eliminate_udp_connection()
                robot.connected_to_hardware = False

    def enable_speed():
        pass

    def enable_steering():
        pass

    def run_trial():
        robot.trial_start_time = get_time_in_ms()
        robot.running_trial    = True
        steering_switch.value  = True
        speed_switch.value     = True
        logging_switch.value   = True

    # ------------------------------------------------------------------ map plot
    _BG      = '#1a1a2e'
    _AX_BG   = '#0f3460'
    _WALL    = 'white'
    _SCAN    = '#666666'
    _LINE_PT = 'cyan'
    _CORNER  = 'red'
    _POSE    = '#00ff88'

    def _draw_walls(ax):
        for seg in parameters.wall_corner_list:
            x1, y1, x2, y2 = seg
            ax.plot([x1, x2], [y1, y2], color=_WALL, lw=2, zorder=1)

    def _world_pts(sensor_pts, pose):
        if len(sensor_pts) == 0:
            return np.empty((0, 2))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        R = np.array([[c, -s], [s, c]])
        return sensor_pts @ R.T + np.array([pose[0], pose[1]])

    def show_map_plot():
        pose = robot.current_pose
        fs   = robot.last_feature_set

        with map_plot:
            fig = map_plot.fig
            fig.clear()
            fig.patch.set_facecolor(_BG)
            ax = fig.add_subplot(111, facecolor=_AX_BG)
            for spine in ax.spines.values():
                spine.set_edgecolor('#444')
            ax.tick_params(colors='lightgray', labelsize=7)

            _draw_walls(ax)

            # raw scan points
            if len(robot.last_scan_pts) > 0:
                wp = _world_pts(robot.last_scan_pts, pose)
                ax.scatter(wp[:, 0], wp[:, 1], s=1, c=_SCAN, alpha=0.5, zorder=2)

            # line feature points
            if fs is not None and len(fs.line_points) > 0:
                wlp = _world_pts(fs.line_points, pose)
                ax.scatter(wlp[:, 0], wlp[:, 1], s=6, c=_LINE_PT, alpha=0.8,
                           zorder=3, label='line pts')

            # corner features
            if fs is not None and len(fs.corners) > 0:
                wcp = _world_pts(fs.corners, pose)
                ax.scatter(wcp[:, 0], wcp[:, 1], s=100, c=_CORNER, marker='x',
                           zorder=4, linewidths=2, label='corners')

            # robot pose
            px, py, pth = pose
            arr_len = 0.12
            ax.scatter([px], [py], s=80, c=_POSE, zorder=5)
            ax.annotate('',
                        xy=(px + arr_len * math.cos(pth),
                            py + arr_len * math.sin(pth)),
                        xytext=(px, py),
                        arrowprops=dict(arrowstyle='->', color=_POSE, lw=2),
                        zorder=6)

            ax.set_aspect('equal')
            ax.grid(True, alpha=0.15, color='gray')
            ax.set_xlabel('x (m)', color='lightgray', fontsize=8)
            ax.set_ylabel('y (m)', color='lightgray', fontsize=8)
            mode_str = robot.debug_info.get('mode', LOCALIZATION_MODE)
            ax.set_title(f'{mode_str.upper()} — Map View', color='white', fontsize=9)
            if fs is not None and (len(fs.corners) > 0 or len(fs.line_points) > 0):
                ax.legend(fontsize=6, facecolor='#222', labelcolor='white',
                          loc='upper right')

    # ------------------------------------------------------------------ debug labels
    def _fmt(val, fmt='.3f'):
        if val is None:
            return '—'
        try:
            return format(val, fmt)
        except (TypeError, ValueError):
            return str(val)

    def update_debug_labels():
        d = robot.debug_info
        mode_lbl.set_text(d.get('mode', LOCALIZATION_MODE))
        conv_lbl.set_text('YES' if d.get('converged') else 'NO')
        conv_lbl.style('color: #00ff88;' if d.get('converged') else 'color: #ff4444;')

        if d.get('mode') == 'wp_icp':
            gc  = d.get('corner_confidence', 0.0)
            gl  = d.get('line_confidence',   0.0)
            gc_lbl.set_text(_fmt(gc))
            gl_lbl.set_text(_fmt(gl))
            nc_lbl.set_text(str(d.get('num_corners',    '—')))
            nl_lbl.set_text(str(d.get('num_line_points','—')))
            ci_lbl.set_text(str(d.get('corner_iterations', '—')))
            li_lbl.set_text(str(d.get('line_iterations',   '—')))
        else:
            gc_lbl.set_text('—')
            gl_lbl.set_text('—')
            nc_lbl.set_text('—')
            nl_lbl.set_text('—')
            ci_lbl.set_text(str(d.get('iterations', '—')))
            li_lbl.set_text('—')

        rt = d.get('runtime_sec')
        rt_lbl.set_text(f'{rt*1000:.1f} ms' if rt is not None else '—')

        p = robot.current_pose
        pose_lbl.set_text(f'x={p[0]:.3f}  y={p[1]:.3f}  θ={math.degrees(p[2]):.1f}°')

    # ================================================================== layout
    with ui.card().classes('w-full items-center'):
        ui.label('ROB-GY 6213 — WP-ICP Localization').style('font-size: 22px;')

    # top row: camera | map plot | controls
    with ui.card().classes('w-full'):
        with ui.grid(columns=3).classes('w-full items-center'):

            # camera / robot image
            with ui.card().classes('w-full items-center h-72'):
                if stream_video and _CV2_AVAILABLE:
                    video_image = ui.interactive_image('/video/frame').classes('w-full h-full')
                else:
                    ui.image('./a_robot_image.jpg').props('height=2')

            # map plot
            with ui.card().classes('w-full items-center h-72'):
                map_plot = ui.pyplot(figsize=(4, 4))

            # controls + debug
            with ui.card().classes('items-center h-72'):
                ui.label('Controls').style('font-weight:bold;')
                encoder_count_label = ui.label('encoder: 0')
                logging_switch  = ui.switch('Data Logging')
                udp_switch      = ui.switch('Robot Connect')
                run_trial_button = ui.button('Run Trial', on_click=lambda: run_trial())

                ui.separator()
                ui.label('Debug').style('font-weight:bold; margin-top:6px;')

                with ui.grid(columns=2):
                    ui.label('Mode:');      mode_lbl = ui.label('—')
                    ui.label('Converged:'); conv_lbl = ui.label('—')
                    ui.label('Γ_C:');       gc_lbl   = ui.label('—')
                    ui.label('Γ_L:');       gl_lbl   = ui.label('—')
                    ui.label('#Corners:');  nc_lbl   = ui.label('—')
                    ui.label('#LinePts:');  nl_lbl   = ui.label('—')
                    ui.label('C-iters:');   ci_lbl   = ui.label('—')
                    ui.label('L-iters:');   li_lbl   = ui.label('—')
                    ui.label('Runtime:');   rt_lbl   = ui.label('—')

                ui.separator()
                pose_lbl = ui.label('x=— y=— θ=—').style('font-size:11px;')

    # speed slider
    with ui.card().classes('w-full'):
        with ui.grid(columns=4).classes('w-full'):
            with ui.card().classes('w-full items-center'):
                ui.label('SPEED:').style('text-align:center;')
            with ui.card().classes('w-full items-center'):
                slider_speed = ui.slider(min=0, max=100, value=0)
            with ui.card().classes('w-full items-center'):
                ui.label().bind_text_from(slider_speed, 'value').style('text-align:center;')
            with ui.card().classes('w-full items-center'):
                speed_switch = ui.switch('Enable', on_change=lambda: enable_speed())

    # steering slider
    with ui.card().classes('w-full'):
        with ui.grid(columns=4).classes('w-full'):
            with ui.card().classes('w-full items-center'):
                ui.label('STEER:').style('text-align:center;')
            with ui.card().classes('w-full items-center'):
                slider_steering = ui.slider(min=-20, max=20, value=0)
            with ui.card().classes('w-full items-center'):
                ui.label().bind_text_from(slider_steering, 'value').style('text-align:center;')
            with ui.card().classes('w-full items-center'):
                steering_switch = ui.switch('Enable', on_change=lambda: enable_steering())

    # ------------------------------------------------------------------ loop
    async def control_loop():
        update_connection()
        cmd_speed, cmd_steer = update_commands()
        robot.control_loop(cmd_speed, cmd_steer, logging_switch.value)
        encoder_count_label.set_text(f'encoder: {robot.robot_sensor_signal.encoder_counts}')
        update_lidar_data()
        show_map_plot()
        update_debug_labels()

    ui.timer(0.1, control_loop)


ui.run(native=True)
