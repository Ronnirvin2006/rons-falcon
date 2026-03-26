#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       slider_teleop.py
# *  Description:    GUI-based joint teleop for the Falcon Robotic Arm.
# *                  Provides an independent tkinter slider for every joint so the
# *                  operator can manually position each joint without a joystick.
# *
# *                  Joint layout:
# *                    Joint 1-5    : arm joints — range ±90° (±1.5708 rad)
# *                    Gripper 1-2  : servo gripper — range -40.1° to 0° (-0.7 to 0 rad)
# *
# *                  Architecture:
# *                    • Main thread   : tkinter GUI (mandatory — not thread-safe)
# *                    • Daemon thread : rclpy.spin(node) for ROS2 callbacks
# *                    • Periodic 100 ms tkinter 'after' timer sends JTC goal when
# *                      any slider has moved since the last send.
# *
# *  Author:         BARGAVAN R - MIT
# *
# *  Dependencies:
# *    ROS2 packages : control_msgs, trajectory_msgs, sensor_msgs, rclpy
# *    Python stdlib  : tkinter (built-in), threading, math
# *
# *  Usage:
# *    ros2 launch falcon_arm_teleop slider_teleop.launch.py
# *    — or —
# *    ros2 run falcon_arm_teleop slider_teleop.py
# *
# *  Notes:
# *    • Works with Gazebo simulation and real hardware alike.
# *    • Controllers must be running before this node is started.
# *    • The node syncs its initial slider positions from /joint_states on startup.
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# Standard-library imports
# ---------------------------------------------------------------------------
import math          # math.degrees() / math.radians() for display ↔ internal conversion
import threading     # threading.Thread to run rclpy.spin in a background daemon thread

# ---------------------------------------------------------------------------
# tkinter — Python's built-in cross-platform GUI toolkit
# ---------------------------------------------------------------------------
import tkinter as tk              # main tkinter namespace (Tk, Frame, Label, etc.)
from tkinter import ttk           # themed widgets (ttk.Scale, ttk.Button for cleaner look)
from tkinter import font as tkfont  # font queries for dynamic sizing

# ---------------------------------------------------------------------------
# ROS2 Python client library
# ---------------------------------------------------------------------------
import rclpy                          # ROS2 init / shutdown
from rclpy.node import Node           # base class for all ROS2 nodes

# ---------------------------------------------------------------------------
# ROS2 message types
# ---------------------------------------------------------------------------
from builtin_interfaces.msg import Duration           # sec + nanosec fields for trajectory
from sensor_msgs.msg import JointState                 # /joint_states feedback
from trajectory_msgs.msg import (
    JointTrajectory,        # container for joint names + trajectory points
    JointTrajectoryPoint,   # single point: positions + time_from_start
)


# ===========================================================================
#  CONSTANTS — joint configuration table
#  Each entry: (display_name, joint_name, min_rad, max_rad, initial_rad)
# ===========================================================================
# fmt: off
JOINT_CONFIG = [
    # Display label         ROS joint name     min (rad)  max (rad)  init (rad)
    # Arm soft limits ±1.50 rad — 4° buffer from URDF hard limits (±1.5708)
    # so the joint never reaches the physics clamp during teleop.
    ('Joint 1  (Base)',     'joint_1',         -1.50,      1.50,      0.0),
    ('Joint 2  (Shoulder)', 'joint_2',         -1.50,      1.50,      0.0),
    ('Joint 3  (Elbow)',    'joint_3',         -1.50,      1.50,      0.0),
    ('Joint 4  (Wrist 1)',  'joint_4',         -1.50,      1.50,      0.30),
    ('Joint 5  (Wrist 2)',  'joint_5',         -1.50,      1.50,      0.0),
    ('Gripper  Left',       'gripper_joint_1', -0.7,       0.0,       0.0),
    ('Gripper  Right',      'gripper_joint_2', -0.7,       0.0,       0.0),
]
# fmt: on

# Joint names that belong to the arm trajectory controller
ARM_JOINT_NAMES = [cfg[1] for cfg in JOINT_CONFIG[:5]]
# Joint names that belong to the gripper position controller
GRIPPER_JOINT_NAMES = [cfg[1] for cfg in JOINT_CONFIG[5:]]

# How far ahead (seconds) the trajectory point is placed relative to now.
# Must be slightly larger than PUBLISH_INTERVAL_MS so the controller always
# has a future target to track — this is what produces smooth, jerk-free motion.
TRAJECTORY_HORIZON_SEC = 0.30

# GUI refresh / publish interval in milliseconds (10 Hz).
PUBLISH_INTERVAL_MS = 100

# Colour scheme for the GUI — matches Falcon brand palette from materials.xacro
COLOR_BG         = '#1e1e2e'   # dark blue-grey background
COLOR_PANEL      = '#2a2a3e'   # slightly lighter panel for each joint row
COLOR_ARM        = '#4da6ff'   # Falcon blue for arm joint sliders
COLOR_GRIPPER    = '#ff9944'   # warm orange for gripper sliders
COLOR_TEXT       = '#e0e0e0'   # light grey text
COLOR_VALUE      = '#ffffff'   # bright white for numeric readout
COLOR_BTN_HOME   = '#3a7bd5'   # blue for Home button
COLOR_BTN_STOP   = '#d53a3a'   # red for Stop / release button

# Step size for the − / + increment buttons (degrees per click).
STEP_DEG = 8.0


# ===========================================================================
#  ROS2 NODE
# ===========================================================================
class FalconSliderTeleopNode(Node):
    """
    ROS2 node for continuous slider teleoperation.

    Publishes JointTrajectory directly to the JTC topic interface instead of
    using the action interface.  This avoids goal queuing, accept/reject
    handshakes, and the dancing behaviour caused by back-to-back action goals.

    The controller receives a rolling horizon point placed TRAJECTORY_HORIZON_SEC
    in the future on every tick — it smoothly tracks the slider position without
    ever finishing a 'goal' and waiting for the next one.
    """

    def __init__(self):
        super().__init__('falcon_slider_teleop')

        # ------------------------------------------------------------------
        # Publisher: arm_controller topic interface
        # JTC listens on <controller_name>/joint_trajectory for direct msgs.
        # ------------------------------------------------------------------
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )

        # ------------------------------------------------------------------
        # Publisher: gripper_controller topic interface
        # ------------------------------------------------------------------
        self._gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10,
        )

        # ------------------------------------------------------------------
        # Internal state: last known joint positions (radians).
        # Seeded from JOINT_CONFIG; overwritten by /joint_states callback.
        # ------------------------------------------------------------------
        self._joint_positions: dict[str, float] = {
            cfg[1]: cfg[4] for cfg in JOINT_CONFIG
        }

        self._joint_states_received: bool = False

        # ------------------------------------------------------------------
        # Subscriber: /joint_states — syncs slider initial positions
        # ------------------------------------------------------------------
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_cb,
            10,
        )

        self.get_logger().info('FalconSliderTeleopNode initialised (topic mode) — waiting for /joint_states')

    # -----------------------------------------------------------------------
    def _joint_states_cb(self, msg: JointState) -> None:
        """Cache latest joint positions from /joint_states."""
        name_idx = {name: i for i, name in enumerate(msg.name)}
        for joint_name in self._joint_positions:
            idx = name_idx.get(joint_name)
            if idx is not None and idx < len(msg.position):
                self._joint_positions[joint_name] = msg.position[idx]

        if not self._joint_states_received:
            self._joint_states_received = True
            self.get_logger().info('Joint states received — sliders synchronised with robot.')

    # -----------------------------------------------------------------------
    def get_initial_positions(self) -> dict[str, float]:
        """Return cached joint positions (called by GUI at startup)."""
        return dict(self._joint_positions)

    # -----------------------------------------------------------------------
    def send_arm_command(self, positions: list[float]) -> None:
        """Publish a rolling-horizon trajectory point to the arm controller."""
        self._publish(self._arm_pub, ARM_JOINT_NAMES, positions)

    # -----------------------------------------------------------------------
    def send_gripper_command(self, positions: list[float]) -> None:
        """Publish a rolling-horizon trajectory point to the gripper controller."""
        self._publish(self._gripper_pub, GRIPPER_JOINT_NAMES, positions)

    # -----------------------------------------------------------------------
    def _publish(
        self,
        pub,
        joint_names: list[str],
        positions: list[float],
    ) -> None:
        """
        Build and publish a single-point JointTrajectory.

        header.stamp MUST be set to the current clock so that the JTC
        interprets time_from_start correctly in sim time.  Without it the
        controller sees a zero-epoch stamp and its timing is undefined,
        which causes it to ignore recovery commands after hitting a limit.

        Positions are clamped to joint limits before publishing so the
        controller never receives an out-of-range command.
        """
        traj = JointTrajectory()
        # Leave header.stamp at zero — JTC treats zero as "use receive time",
        # which is always correct regardless of sim vs wall clock.
        # Setting it to node clock breaks Gazebo (wall time >> sim time).
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        # Clamp each position to [min_rad, max_rad] from JOINT_CONFIG
        clamped = []
        for name, pos in zip(joint_names, positions):
            cfg = next(c for c in JOINT_CONFIG if c[1] == name)
            clamped.append(max(cfg[2], min(cfg[3], pos)))
        point.positions = clamped

        sec = int(TRAJECTORY_HORIZON_SEC)
        nanosec = int((TRAJECTORY_HORIZON_SEC - sec) * 1_000_000_000)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points = [point]
        pub.publish(traj)


# ===========================================================================
#  tkinter GUI
# ===========================================================================
class SliderGUI:
    """
    Full tkinter GUI for manual joint control.

    Window layout:
        Title bar
        ┌─────────────────────────────────────────────────┐
        │  Joint 1  (Base)   [slider track]   value label  │
        │  Joint 2  (Shoulder)  ...                        │
        │  ...                                             │
        │  Gripper Left  ...                               │
        │  Gripper Right ...                               │
        ├─────────────────────────────────────────────────┤
        │  [Home All]   [Open Gripper]   [Close Gripper]   │
        └─────────────────────────────────────────────────┘
    """

    def __init__(self, root: tk.Tk, node: FalconSliderTeleopNode):
        """
        Initialise the GUI and wire up all widgets.

        Parameters
        ----------
        root  : The top-level tkinter window.
        node  : Running ROS2 node for sending trajectory goals.
        """
        self._root = root          # store reference to Tk root
        self._node = node          # ROS2 node for publishing goals

        # ------------------------------------------------------------------
        # Configure root window appearance
        # ------------------------------------------------------------------
        root.title('Falcon Arm — Joint Slider Teleop')
        root.configure(bg=COLOR_BG)           # dark background
        root.resizable(False, False)           # fixed size — cleaner look

        # ------------------------------------------------------------------
        # Per-joint tkinter DoubleVar objects.
        # DoubleVar automatically triggers trace callbacks when the slider moves.
        # Stored in order matching JOINT_CONFIG.
        # ------------------------------------------------------------------
        self._vars: list[tk.DoubleVar] = []

        # ------------------------------------------------------------------
        # Per-joint label widgets that display the current numeric value.
        # Updated every PUBLISH_INTERVAL_MS by _refresh_labels().
        # ------------------------------------------------------------------
        self._value_labels: list[tk.Label] = []

        # ------------------------------------------------------------------
        # Build widgets
        # ------------------------------------------------------------------
        self._build_header()
        self._build_joint_rows()
        self._build_control_buttons()

        # ------------------------------------------------------------------
        # Sync sliders with robot's actual joint positions after a short delay.
        # The 500 ms wait gives the ROS2 spin thread time to receive /joint_states.
        # ------------------------------------------------------------------
        root.after(500, self._sync_from_robot)

        # ------------------------------------------------------------------
        # Start the periodic publish + label refresh loop
        # ------------------------------------------------------------------
        root.after(PUBLISH_INTERVAL_MS, self._periodic_tick)

    # -----------------------------------------------------------------------
    def _build_header(self) -> None:
        """Create the title banner at the top of the window."""
        header = tk.Frame(self._root, bg=COLOR_BG)
        header.pack(fill='x', padx=10, pady=(10, 4))

        tk.Label(
            header,
            text='FALCON ROBOTIC ARM — Joint Controller',
            bg=COLOR_BG,
            fg=COLOR_ARM,
            font=('Courier', 14, 'bold'),
        ).pack(side='left')

        # Subtitle / status line
        self._status_var = tk.StringVar(value='Connecting to ROS2...')
        tk.Label(
            header,
            textvariable=self._status_var,   # dynamic text
            bg=COLOR_BG,
            fg=COLOR_TEXT,
            font=('Courier', 9),
        ).pack(side='right')

    # -----------------------------------------------------------------------
    def _build_joint_rows(self) -> None:
        """
        Create one row per joint in JOINT_CONFIG.
        Each row contains:
          - A joint label  (left)
          - A horizontal Scale slider  (centre)
          - A numeric readout in degrees  (right)
        Arm joints use blue accent; gripper joints use orange accent.
        """
        for i, (label_text, joint_name, min_rad, max_rad, init_rad) in enumerate(JOINT_CONFIG):

            # Decide accent colour by group
            is_gripper = joint_name.startswith('gripper')
            accent = COLOR_GRIPPER if is_gripper else COLOR_ARM

            # ------------------------------------------------------------------
            # Outer frame for this joint row
            # ------------------------------------------------------------------
            row = tk.Frame(
                self._root,
                bg=COLOR_PANEL,
                relief='flat',
                bd=0,
            )
            row.pack(fill='x', padx=10, pady=3, ipady=4)

            # ------------------------------------------------------------------
            # Joint name label  (fixed width for column alignment)
            # ------------------------------------------------------------------
            tk.Label(
                row,
                text=label_text,
                bg=COLOR_PANEL,
                fg=accent,
                font=('Courier', 10, 'bold'),
                width=22,          # chars — keeps sliders in a clean column
                anchor='w',        # left-align text
            ).pack(side='left', padx=(8, 4))

            # ------------------------------------------------------------------
            # DoubleVar: stores slider value in DEGREES for display convenience.
            # We convert to radians only when sending the goal.
            # ------------------------------------------------------------------
            var = tk.DoubleVar(value=math.degrees(init_rad))
            self._vars.append(var)

            # ------------------------------------------------------------------
            # Horizontal Scale widget
            # ttk.Scale for modern flat look; from_ / to in DEGREES
            # ------------------------------------------------------------------
            slider = ttk.Scale(
                row,
                orient='horizontal',
                from_=math.degrees(min_rad),   # convert rad limits to degrees
                to=math.degrees(max_rad),
                variable=var,                  # live-linked DoubleVar
                length=320,                    # pixel width of the slider track
            )
            # Apply trough colour via style (ttk theming)
            style_name = f'Accent{i}.Horizontal.TScale'
            style = ttk.Style()
            style.theme_use('clam')            # 'clam' supports colour overrides
            style.configure(
                style_name,
                troughcolor=COLOR_PANEL,       # track background
                background=accent,             # slider thumb colour
            )
            slider.configure(style=style_name)
            # Disable mouse interaction — position indicator only, moved by +/- buttons.
            slider.configure(state='disabled')

            # Gripper rows: no +/- buttons, slider indicator only.
            # Arm rows: flanking −/+ buttons, 8° step, slider is read-only indicator.
            if not is_gripper:
                min_deg_val = math.degrees(min_rad)
                max_deg_val = math.degrees(max_rad)

                tk.Button(
                    row,
                    text='−',
                    bg=COLOR_PANEL,
                    fg=accent,
                    font=('Courier', 10, 'bold'),
                    relief='flat',
                    width=2,
                    command=lambda v=var, lo=min_deg_val: v.set(max(lo, v.get() - STEP_DEG)),
                ).pack(side='left', padx=(0, 2))

            slider.pack(side='left', padx=4)

            if not is_gripper:
                tk.Button(
                    row,
                    text='+',
                    bg=COLOR_PANEL,
                    fg=accent,
                    font=('Courier', 10, 'bold'),
                    relief='flat',
                    width=2,
                    command=lambda v=var, hi=max_deg_val: v.set(min(hi, v.get() + STEP_DEG)),
                ).pack(side='left', padx=(2, 0))

            # ------------------------------------------------------------------
            # Numeric readout label:  e.g.  " 45.2°  /  0.789 rad "
            # ------------------------------------------------------------------
            val_label = tk.Label(
                row,
                text=self._format_value(init_rad),   # initial text
                bg=COLOR_PANEL,
                fg=COLOR_VALUE,
                font=('Courier', 9),
                width=22,    # fixed width prevents window resize jitter
                anchor='e',  # right-align
            )
            val_label.pack(side='left', padx=(4, 8))
            self._value_labels.append(val_label)

    # -----------------------------------------------------------------------
    def _build_control_buttons(self) -> None:
        """
        Create the bottom control bar with Home, Open Gripper, Close Gripper buttons.
        """
        bar = tk.Frame(self._root, bg=COLOR_BG)
        bar.pack(fill='x', padx=10, pady=(6, 10))

        # Home All — moves all joints to their defined home positions
        tk.Button(
            bar,
            text='Home All',
            bg=COLOR_BTN_HOME,
            fg='white',
            font=('Courier', 10, 'bold'),
            relief='flat',
            padx=10,
            command=self._home_all,           # callback resets all sliders
        ).pack(side='left', padx=4)

        # "Close Gripper" label → calls _open_gripper (joints to 0 rad, fingers spread)
        tk.Button(
            bar,
            text='Close Gripper',
            bg='#2e7d32',           # dark green
            fg='white',
            font=('Courier', 10, 'bold'),
            relief='flat',
            padx=10,
            command=self._open_gripper,
        ).pack(side='left', padx=4)

        # "Open Gripper" label → calls _close_gripper (joints to -0.7 rad, fingers closed)
        tk.Button(
            bar,
            text='Open Gripper',
            bg=COLOR_BTN_STOP,      # red
            fg='white',
            font=('Courier', 10, 'bold'),
            relief='flat',
            padx=10,
            command=self._close_gripper,
        ).pack(side='left', padx=4)

    # -----------------------------------------------------------------------
    #  Callbacks
    # -----------------------------------------------------------------------

    def _periodic_tick(self) -> None:
        """
        Runs every PUBLISH_INTERVAL_MS milliseconds in the main (GUI) thread.

        Always publishes current slider positions — the controller needs a
        continuous stream of rolling-horizon targets to track smoothly.
        Removing the dirty-flag gate means the arm holds its position when
        the slider is untouched (controller keeps re-receiving the same point)
        instead of stopping and waiting for the next action goal.
        """
        self._refresh_labels()

        # Read all slider values (degrees) and convert to radians
        positions_rad = [
            math.radians(self._vars[i].get())
            for i in range(len(JOINT_CONFIG))
        ]

        self._node.send_arm_command(positions_rad[:5])
        self._node.send_gripper_command(positions_rad[5:])

        # Reschedule — keeps the loop alive
        self._root.after(PUBLISH_INTERVAL_MS, self._periodic_tick)

    # -----------------------------------------------------------------------
    def _refresh_labels(self) -> None:
        """Update every numeric readout label with the current slider value."""
        for i, (_, _, _, _, _) in enumerate(JOINT_CONFIG):
            deg = self._vars[i].get()                        # slider value in degrees
            rad = math.radians(deg)                          # convert to radians for display
            self._value_labels[i].config(
                text=self._format_value(rad)                 # update label text
            )

    # -----------------------------------------------------------------------
    def _sync_from_robot(self) -> None:
        """
        One-time startup sync: reads current joint positions from the ROS2 node
        (which caches them from /joint_states) and sets sliders to match.
        Called 500 ms after GUI startup to give time for first /joint_states msg.
        """
        positions = self._node.get_initial_positions()   # dict: name → rad

        for i, (_, joint_name, _, _, _) in enumerate(JOINT_CONFIG):
            if joint_name in positions:
                deg = math.degrees(positions[joint_name])  # convert rad → degrees for slider
                self._vars[i].set(deg)                     # move slider to robot's position

        self._status_var.set('Connected — sliders synced with robot.')

    # -----------------------------------------------------------------------
    #  Button commands
    # -----------------------------------------------------------------------

    def _home_all(self) -> None:
        """Reset every joint slider to its defined home position from JOINT_CONFIG."""
        for i, (_, _, _, _, init_rad) in enumerate(JOINT_CONFIG):
            self._vars[i].set(math.degrees(init_rad))

    def _open_gripper(self) -> None:
        """Set both gripper sliders to 0° (fully open — max_rad for gripper = 0.0)."""
        self._vars[5].set(0.0)   # gripper_joint_1 → 0 deg = 0.0 rad
        self._vars[6].set(0.0)   # gripper_joint_2 → 0 deg = 0.0 rad

    def _close_gripper(self) -> None:
        """Set both gripper sliders to their most-closed position (-0.7 rad → -40.1°)."""
        self._vars[5].set(math.degrees(-0.7))   # gripper_joint_1 minimum
        self._vars[6].set(math.degrees(-0.7))   # gripper_joint_2 minimum

    # -----------------------------------------------------------------------
    #  Static helper
    # -----------------------------------------------------------------------

    @staticmethod
    def _format_value(rad: float) -> str:
        """
        Format a radian value for display in the value label.
        Returns a fixed-width string like:  ' 45.2°  /  0.789 rad'
        """
        deg = math.degrees(rad)
        return f'{deg:+7.1f}°  /  {rad:+.3f} rad'


# ===========================================================================
#  ENTRY POINT
# ===========================================================================
def main(args=None) -> None:
    """
    Application entry point.

    Execution model:
        1. Initialise rclpy and create the ROS2 node.
        2. Spin the ROS2 node in a daemon thread so it processes callbacks
           independently of the GUI.  Daemon=True ensures the thread is killed
           automatically when the main (GUI) thread exits.
        3. Build the tkinter GUI on the main thread and start its event loop.
        4. On GUI close, stop the spin thread cleanly and shut down rclpy.
    """
    # Initialise ROS2 Python client library with optional CLI args
    rclpy.init(args=args)

    # Instantiate the ROS2 node (action clients + subscriber)
    node = FalconSliderTeleopNode()

    # ------------------------------------------------------------------
    # Start rclpy spin in a background daemon thread.
    # daemon=True: the thread is auto-killed when the main thread ends,
    # preventing a hang if the GUI is closed with Ctrl+C or the X button.
    # ------------------------------------------------------------------
    spin_thread = threading.Thread(
        target=rclpy.spin,   # function to run: spin the node event loop
        args=(node,),         # pass the node to spin
        daemon=True,          # auto-exit when main thread ends
    )
    spin_thread.start()

    # ------------------------------------------------------------------
    # Build the tkinter root window and attach the GUI application
    # ------------------------------------------------------------------
    root = tk.Tk()
    app = SliderGUI(root, node)   # builds all widgets, starts periodic tick

    # Set a minimum window size to prevent widget clipping
    root.update_idletasks()       # ensure all widgets are sized before querying
    root.minsize(
        root.winfo_reqwidth(),    # minimum width = natural widget width
        root.winfo_reqheight(),   # minimum height = natural widget height
    )

    # ------------------------------------------------------------------
    # Run the tkinter main event loop (blocks until window is closed)
    # ------------------------------------------------------------------
    try:
        root.mainloop()
    finally:
        # ------------------------------------------------------------------
        # Clean up ROS2 resources after the GUI exits
        # ------------------------------------------------------------------
        node.destroy_node()   # unregister node from ROS2 graph
        rclpy.shutdown()       # shut down rclpy (signals spin thread to stop)


if __name__ == '__main__':
    main()
