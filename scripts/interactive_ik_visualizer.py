#!/usr/bin/env python3
"""
Interactive IK Visualizer.

Plotly Dash app for visualizing IK round-trip error:
- Blue robot: Baseline configuration (set via joint sliders)
- Red robot: IK solution for target end-effector position
- Shows IK error: ||FK(IK(X_cmd)) - X_cmd||

Usage:
    cd /path/to/so101_ik_fk
    python scripts/interactive_ik_visualizer.py
    # Open http://127.0.0.1:8050 in browser
"""

from __future__ import annotations

import sys
from pathlib import Path

# Add parent directory to path for running as script
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, callback_context
from dash.dependencies import Input, Output, State
from dash.exceptions import PreventUpdate

from lib.so101_kinematics import (
    SO101ForwardKinematics,
    SO101Position,
    SO101_JOINT_NAMES,
    DEFAULT_SO101_URDF_PATH,
)
from metrics.error_metrics import KinematicErrorMetrics


# =============================================================================
# Robot Link Visualization Helper
# =============================================================================

class RobotVisualizer:
    """Helper to compute robot link positions for visualization."""

    # Links to visualize (in order from base to EE)
    LINK_NAMES = [
        "base_link",
        "shoulder_link",
        "upper_arm_link",
        "lower_arm_link",
        "wrist_link",
        "gripper_link",
        "gripper_frame_link",
    ]

    def __init__(self, urdf_path: str | Path | None = None):
        """Initialize visualizer with URDF."""
        try:
            import placo
        except ImportError as e:
            raise ImportError("placo is required for visualization") from e

        if urdf_path is None:
            urdf_path = DEFAULT_SO101_URDF_PATH

        self.robot = placo.RobotWrapper(str(urdf_path))
        self.joint_names = SO101_JOINT_NAMES

    def get_link_positions(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """
        Get 3D positions of all links for given joint configuration.

        Args:
            joint_angles_deg: 5 joint angles in degrees

        Returns:
            (N, 3) array of link positions
        """
        joint_rad = np.deg2rad(joint_angles_deg[:len(self.joint_names)])

        for i, name in enumerate(self.joint_names):
            self.robot.set_joint(name, joint_rad[i])
        self.robot.update_kinematics()

        positions = []
        for link_name in self.LINK_NAMES:
            T = self.robot.get_T_world_frame(link_name)
            positions.append(T[:3, 3])

        return np.array(positions)


# =============================================================================
# Dash App
# =============================================================================

# Initialize FK and metrics
fk = SO101ForwardKinematics()
viz = RobotVisualizer()
metrics = KinematicErrorMetrics(
    fk_fn=fk.get_ee_position,
    ik_fn=fk.kinematics.inverse_kinematics,
)

# Joint limits in degrees
JOINT_LIMITS = {
    "shoulder_pan": (-110, 110),
    "shoulder_lift": (-100, 100),
    "elbow_flex": (-96, 96),
    "wrist_flex": (-95, 95),
    "wrist_roll": (-157, 163),
}

# Create Dash app
app = Dash(__name__)
app.title = "SO101 IK Visualizer"

# CSS for dark theme
DARK_STYLE = {
    "backgroundColor": "#1a1a2e",
    "color": "#eee",
    "fontFamily": "'JetBrains Mono', 'Fira Code', monospace",
    "padding": "20px",
    "minHeight": "100vh",
}

SLIDER_STYLE = {
    "marginBottom": "25px",
}

LABEL_ROW_STYLE = {
    "display": "flex",
    "alignItems": "center",
    "marginBottom": "5px",
}

JOINT_LABEL_STYLE = {
    "width": "130px",
    "textAlign": "right",
    "paddingRight": "10px",
}

JOINT_VALUE_STYLE = {
    "color": "#e94560",
    "width": "50px",
    "textAlign": "left",
}

CARD_STYLE = {
    "backgroundColor": "#16213e",
    "borderRadius": "8px",
    "padding": "20px",
    "marginBottom": "20px",
    "border": "1px solid #0f3460",
}

ERROR_STYLE = {
    "fontSize": "28px",
    "fontWeight": "bold",
    "textAlign": "center",
    "padding": "15px",
    "borderRadius": "8px",
    "marginBottom": "20px",
}

# App layout
app.layout = html.Div(style=DARK_STYLE, children=[
    html.H1(
        "SO101 Interactive IK Visualizer",
        style={"textAlign": "center", "color": "#e94560", "marginBottom": "10px"}
    ),
    html.P(
        "Blue = Baseline Robot | Red = IK Solution | Green = Target EE",
        style={"textAlign": "center", "color": "#888", "marginBottom": "30px"}
    ),

    # IK Error display
    html.Div(id="ik-error-display", style=ERROR_STYLE),

    html.Div(style={"display": "flex", "gap": "30px"}, children=[
        # Left panel: Controls
        html.Div(style={"flex": "0 0 380px"}, children=[
            # Joint angle controls
            html.Div(style=CARD_STYLE, children=[
                html.H3("Joint Angles (Baseline)", style={"color": "#0f4c75", "marginTop": "0"}),
                *[
                    html.Div(style=SLIDER_STYLE, children=[
                        html.Div(style=LABEL_ROW_STYLE, children=[
                            html.Label(f"{name}:", style=JOINT_LABEL_STYLE),
                            html.Span(id=f"joint-{i}-value", style=JOINT_VALUE_STYLE),
                        ]),
                        dcc.Slider(
                            id=f"joint-{i}",
                            min=JOINT_LIMITS[name][0],
                            max=JOINT_LIMITS[name][1],
                            step=1,
                            value=0,
                            marks={JOINT_LIMITS[name][0]: str(JOINT_LIMITS[name][0]),
                                   0: "0",
                                   JOINT_LIMITS[name][1]: str(JOINT_LIMITS[name][1])},
                            tooltip={"placement": "bottom"},
                        ),
                    ])
                    for i, name in enumerate(SO101_JOINT_NAMES)
                ],
                html.Button(
                    "Reset to Home",
                    id="reset-joints",
                    style={"width": "100%", "padding": "10px", "marginTop": "10px",
                           "backgroundColor": "#0f4c75", "color": "white", "border": "none",
                           "borderRadius": "4px", "cursor": "pointer"}
                ),
            ]),

            # Target EE controls
            html.Div(style=CARD_STYLE, children=[
                html.H3("Target End-Effector (X_cmd)", style={"color": "#e94560", "marginTop": "0"}),

                # Mode indicator
                html.Div(id="edit-mode-indicator", style={
                    "padding": "8px", "borderRadius": "4px", "marginBottom": "15px",
                    "textAlign": "center", "fontWeight": "bold"
                }),

                # Store for edit mode state
                dcc.Store(id="edit-mode", data=False),

                html.Div(style=SLIDER_STYLE, children=[
                    html.Div(style=LABEL_ROW_STYLE, children=[
                        html.Label("X:", style={**JOINT_LABEL_STYLE, "width": "30px"}),
                        html.Span(id="ee-x-value", style={**JOINT_VALUE_STYLE, "color": "#4ecca3", "width": "80px"}),
                    ]),
                    dcc.Slider(
                        id="ee-x",
                        min=-0.3, max=0.4, step=0.005, value=0.2,
                        marks={-0.3: "-0.3m", 0: "0", 0.4: "0.4m"},
                        tooltip={"placement": "bottom"},
                        disabled=True,
                    ),
                ]),
                html.Div(style=SLIDER_STYLE, children=[
                    html.Div(style=LABEL_ROW_STYLE, children=[
                        html.Label("Y:", style={**JOINT_LABEL_STYLE, "width": "30px"}),
                        html.Span(id="ee-y-value", style={**JOINT_VALUE_STYLE, "color": "#4ecca3", "width": "80px"}),
                    ]),
                    dcc.Slider(
                        id="ee-y",
                        min=-0.3, max=0.3, step=0.005, value=0.0,
                        marks={-0.3: "-0.3m", 0: "0", 0.3: "0.3m"},
                        tooltip={"placement": "bottom"},
                        disabled=True,
                    ),
                ]),
                html.Div(style=SLIDER_STYLE, children=[
                    html.Div(style=LABEL_ROW_STYLE, children=[
                        html.Label("Z:", style={**JOINT_LABEL_STYLE, "width": "30px"}),
                        html.Span(id="ee-z-value", style={**JOINT_VALUE_STYLE, "color": "#4ecca3", "width": "80px"}),
                    ]),
                    dcc.Slider(
                        id="ee-z",
                        min=-0.1, max=0.4, step=0.005, value=0.15,
                        marks={-0.1: "-0.1m", 0: "0", 0.4: "0.4m"},
                        tooltip={"placement": "bottom"},
                        disabled=True,
                    ),
                ]),

                # Buttons for edit mode
                html.Div(style={"display": "flex", "gap": "10px", "marginTop": "10px"}, children=[
                    html.Button(
                        "Set End Effector Position",
                        id="btn-start-edit",
                        style={"flex": "1", "padding": "12px",
                               "backgroundColor": "#4ecca3", "color": "#1a1a2e", "border": "none",
                               "borderRadius": "4px", "cursor": "pointer", "fontWeight": "bold"}
                    ),
                    html.Button(
                        "Done",
                        id="btn-done-edit",
                        style={"flex": "1", "padding": "12px",
                               "backgroundColor": "#666", "color": "white", "border": "none",
                               "borderRadius": "4px", "cursor": "not-allowed"},
                        disabled=True,
                    ),
                ]),

                # Sync to baseline button
                html.Button(
                    "Snap to Baseline EE",
                    id="sync-ee",
                    style={"width": "100%", "padding": "10px", "marginTop": "10px",
                           "backgroundColor": "#0f4c75", "color": "white", "border": "none",
                           "borderRadius": "4px", "cursor": "not-allowed"},
                    disabled=True,
                ),
            ]),

            # Info panel
            html.Div(style=CARD_STYLE, children=[
                html.H4("Current Values", style={"color": "#4ecca3", "marginTop": "0"}),
                html.Div(id="info-panel", style={"fontFamily": "monospace", "fontSize": "12px"}),
            ]),
        ]),

        # Right panel: 3D Plot
        html.Div(style={"flex": "1"}, children=[
            dcc.Graph(
                id="robot-plot",
                style={"height": "80vh"},
                config={"displayModeBar": True, "scrollZoom": True},
            ),
        ]),
    ]),
])


# =============================================================================
# Callbacks
# =============================================================================

@app.callback(
    [Output(f"joint-{i}-value", "children") for i in range(5)],
    [Input(f"joint-{i}", "value") for i in range(5)],
)
def update_joint_labels(*joint_values):
    """Update joint angle display values."""
    return [f"{v}°" for v in joint_values]


@app.callback(
    [Output("ee-x-value", "children"),
     Output("ee-y-value", "children"),
     Output("ee-z-value", "children")],
    [Input("ee-x", "value"),
     Input("ee-y", "value"),
     Input("ee-z", "value")],
)
def update_ee_labels(x, y, z):
    """Update EE position display values."""
    return f"{x:.3f}m", f"{y:.3f}m", f"{z:.3f}m"


@app.callback(
    [Output(f"joint-{i}", "value") for i in range(5)],
    Input("reset-joints", "n_clicks"),
    prevent_initial_call=True,
)
def reset_joints(n_clicks):
    """Reset joints to home position."""
    return [0, 0, 0, 0, 0]


@app.callback(
    Output("edit-mode", "data"),
    [Input("btn-start-edit", "n_clicks"),
     Input("btn-done-edit", "n_clicks")],
    State("edit-mode", "data"),
    prevent_initial_call=True,
)
def toggle_edit_mode(start_clicks, done_clicks, current_mode):
    """Toggle edit mode on/off."""
    ctx = callback_context
    if not ctx.triggered:
        raise PreventUpdate

    trigger_id = ctx.triggered[0]["prop_id"].split(".")[0]

    if trigger_id == "btn-start-edit":
        return True
    elif trigger_id == "btn-done-edit":
        return False

    return current_mode


@app.callback(
    [Output("ee-x", "disabled"),
     Output("ee-y", "disabled"),
     Output("ee-z", "disabled"),
     Output("sync-ee", "disabled"),
     Output("btn-start-edit", "disabled"),
     Output("btn-done-edit", "disabled"),
     Output("btn-start-edit", "style"),
     Output("btn-done-edit", "style"),
     Output("sync-ee", "style"),
     Output("edit-mode-indicator", "children"),
     Output("edit-mode-indicator", "style")],
    Input("edit-mode", "data"),
)
def update_edit_mode_ui(edit_mode):
    """Update UI elements based on edit mode."""
    if edit_mode:
        # Edit mode ON
        return (
            False, False, False,  # Sliders enabled
            False,  # Sync button enabled
            True,   # Start button disabled
            False,  # Done button enabled
            {"flex": "1", "padding": "12px", "backgroundColor": "#444", "color": "#888",
             "border": "none", "borderRadius": "4px", "cursor": "not-allowed"},
            {"flex": "1", "padding": "12px", "backgroundColor": "#e94560", "color": "white",
             "border": "none", "borderRadius": "4px", "cursor": "pointer", "fontWeight": "bold"},
            {"width": "100%", "padding": "10px", "marginTop": "10px",
             "backgroundColor": "#0f4c75", "color": "white", "border": "none",
             "borderRadius": "4px", "cursor": "pointer"},
            "EDITING MODE - Move sliders or click plot",
            {"padding": "8px", "borderRadius": "4px", "marginBottom": "15px",
             "textAlign": "center", "fontWeight": "bold",
             "backgroundColor": "rgba(78, 204, 163, 0.3)", "color": "#4ecca3",
             "border": "2px solid #4ecca3"}
        )
    else:
        # Edit mode OFF
        return (
            True, True, True,  # Sliders disabled
            True,   # Sync button disabled
            False,  # Start button enabled
            True,   # Done button disabled
            {"flex": "1", "padding": "12px", "backgroundColor": "#4ecca3", "color": "#1a1a2e",
             "border": "none", "borderRadius": "4px", "cursor": "pointer", "fontWeight": "bold"},
            {"flex": "1", "padding": "12px", "backgroundColor": "#444", "color": "#888",
             "border": "none", "borderRadius": "4px", "cursor": "not-allowed"},
            {"width": "100%", "padding": "10px", "marginTop": "10px",
             "backgroundColor": "#333", "color": "#666", "border": "none",
             "borderRadius": "4px", "cursor": "not-allowed"},
            "Position LOCKED - Click 'Set End Effector Position' to edit",
            {"padding": "8px", "borderRadius": "4px", "marginBottom": "15px",
             "textAlign": "center", "fontWeight": "bold",
             "backgroundColor": "rgba(102, 102, 102, 0.3)", "color": "#888",
             "border": "2px solid #666"}
        )


@app.callback(
    [Output("ee-x", "value"),
     Output("ee-y", "value"),
     Output("ee-z", "value")],
    [Input("sync-ee", "n_clicks"),
     Input("robot-plot", "clickData")],
    [State(f"joint-{i}", "value") for i in range(5)] + [State("edit-mode", "data")],
    prevent_initial_call=True,
)
def update_target_ee(sync_clicks, click_data, *args):
    """Set target EE position from button or 3D plot click."""
    joint_values = args[:5]
    edit_mode = args[5]

    ctx = callback_context
    if not ctx.triggered:
        raise PreventUpdate

    trigger_id = ctx.triggered[0]["prop_id"].split(".")[0]

    if trigger_id == "sync-ee" and edit_mode:
        # Sync button clicked - set to baseline EE
        joints = np.array(joint_values, dtype=float)
        ee_pos = fk.get_ee_position(joints)
        return float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])

    elif trigger_id == "robot-plot" and click_data and edit_mode:
        # 3D plot clicked - extract coordinates
        point = click_data.get("points", [{}])[0]
        x = point.get("x", 0.2)
        y = point.get("y", 0.0)
        z = point.get("z", 0.15)
        # Clamp to slider ranges
        x = max(-0.3, min(0.4, x))
        y = max(-0.3, min(0.3, y))
        z = max(-0.1, min(0.4, z))
        return float(x), float(y), float(z)

    raise PreventUpdate


@app.callback(
    [Output("robot-plot", "figure"),
     Output("ik-error-display", "children"),
     Output("ik-error-display", "style"),
     Output("info-panel", "children")],
    [Input(f"joint-{i}", "value") for i in range(5)] +
    [Input("ee-x", "value"),
     Input("ee-y", "value"),
     Input("ee-z", "value")],
)
def update_plot(*args):
    """Update 3D plot and error display."""
    joint_values = args[:5]
    ee_x, ee_y, ee_z = args[5:]

    # Baseline joint configuration
    theta_baseline = np.array(joint_values, dtype=float)

    # Target EE position
    X_cmd = np.array([ee_x, ee_y, ee_z])

    # Get baseline robot positions
    baseline_positions = viz.get_link_positions(theta_baseline)
    baseline_ee = baseline_positions[-1]

    # Build target pose matrix for IK
    target_pose = np.eye(4)
    target_pose[:3, 3] = X_cmd

    # Solve IK
    theta_ik = fk.kinematics.inverse_kinematics(theta_baseline, target_pose)

    # Get IK solution robot positions
    ik_positions = viz.get_link_positions(theta_ik)
    ik_ee = ik_positions[-1]

    # Compute IK error
    ik_error = np.linalg.norm(ik_ee - X_cmd)
    ik_error_mm = ik_error * 1000

    # Create 3D figure
    fig = go.Figure()

    # Baseline robot (Blue)
    fig.add_trace(go.Scatter3d(
        x=baseline_positions[:, 0],
        y=baseline_positions[:, 1],
        z=baseline_positions[:, 2],
        mode="lines+markers",
        name="Baseline Robot",
        line=dict(color="#4a90d9", width=8),
        marker=dict(size=6, color="#4a90d9"),
    ))

    # Baseline EE marker
    fig.add_trace(go.Scatter3d(
        x=[baseline_ee[0]],
        y=[baseline_ee[1]],
        z=[baseline_ee[2]],
        mode="markers",
        name="Baseline EE",
        marker=dict(size=12, color="#4a90d9", symbol="diamond"),
    ))

    # IK solution robot (Red)
    fig.add_trace(go.Scatter3d(
        x=ik_positions[:, 0],
        y=ik_positions[:, 1],
        z=ik_positions[:, 2],
        mode="lines+markers",
        name="IK Solution",
        line=dict(color="#e94560", width=8),
        marker=dict(size=6, color="#e94560"),
    ))

    # IK EE marker (actual achieved)
    fig.add_trace(go.Scatter3d(
        x=[ik_ee[0]],
        y=[ik_ee[1]],
        z=[ik_ee[2]],
        mode="markers",
        name="IK EE (achieved)",
        marker=dict(size=12, color="#e94560", symbol="diamond"),
    ))

    # Target EE marker (commanded)
    fig.add_trace(go.Scatter3d(
        x=[X_cmd[0]],
        y=[X_cmd[1]],
        z=[X_cmd[2]],
        mode="markers",
        name="Target EE (X_cmd)",
        marker=dict(size=14, color="#4ecca3", symbol="x"),
    ))

    # Error line (from achieved to target)
    if ik_error > 0.0001:
        fig.add_trace(go.Scatter3d(
            x=[ik_ee[0], X_cmd[0]],
            y=[ik_ee[1], X_cmd[1]],
            z=[ik_ee[2], X_cmd[2]],
            mode="lines",
            name=f"IK Error ({ik_error_mm:.2f}mm)",
            line=dict(color="#ff6b6b", width=4, dash="dash"),
        ))

    # ==========================================================================
    # Mini coordinate axes at robot base (origin)
    # ==========================================================================
    axis_length = 0.08  # 8cm axes
    axis_width = 6

    # X-axis (Red)
    fig.add_trace(go.Scatter3d(
        x=[0, axis_length], y=[0, 0], z=[0, 0],
        mode="lines+text",
        name="X-axis",
        line=dict(color="#ff4444", width=axis_width),
        text=["", "X"],
        textposition="top center",
        textfont=dict(size=14, color="#ff4444"),
        showlegend=False,
    ))

    # Y-axis (Green)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, axis_length], z=[0, 0],
        mode="lines+text",
        name="Y-axis",
        line=dict(color="#44ff44", width=axis_width),
        text=["", "Y"],
        textposition="top center",
        textfont=dict(size=14, color="#44ff44"),
        showlegend=False,
    ))

    # Z-axis (Blue)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, 0], z=[0, axis_length],
        mode="lines+text",
        name="Z-axis",
        line=dict(color="#4444ff", width=axis_width),
        text=["", "Z"],
        textposition="top center",
        textfont=dict(size=14, color="#4444ff"),
        showlegend=False,
    ))

    # Origin marker
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode="markers",
        name="Origin",
        marker=dict(size=8, color="#ffffff", symbol="circle"),
        showlegend=False,
    ))

    # Layout - positive quadrant focus
    fig.update_layout(
        scene=dict(
            xaxis=dict(title="X (m)", range=[-0.05, 0.5], backgroundcolor="#1a1a2e", gridcolor="#333"),
            yaxis=dict(title="Y (m)", range=[-0.25, 0.25], backgroundcolor="#1a1a2e", gridcolor="#333"),
            zaxis=dict(title="Z (m)", range=[-0.05, 0.5], backgroundcolor="#1a1a2e", gridcolor="#333"),
            aspectmode="cube",
            camera=dict(eye=dict(x=1.8, y=1.2, z=0.8)),
        ),
        paper_bgcolor="#1a1a2e",
        plot_bgcolor="#1a1a2e",
        font=dict(color="#eee"),
        legend=dict(
            yanchor="top", y=0.99,
            xanchor="left", x=0.01,
            bgcolor="rgba(22, 33, 62, 0.8)",
        ),
        margin=dict(l=0, r=0, t=30, b=0),
    )

    # Error display styling
    if ik_error_mm < 1.0:
        error_color = "#4ecca3"  # Green - good
        error_bg = "rgba(78, 204, 163, 0.2)"
    elif ik_error_mm < 5.0:
        error_color = "#f9ed69"  # Yellow - acceptable
        error_bg = "rgba(249, 237, 105, 0.2)"
    else:
        error_color = "#e94560"  # Red - bad
        error_bg = "rgba(233, 69, 96, 0.2)"

    error_style = {
        **ERROR_STYLE,
        "color": error_color,
        "backgroundColor": error_bg,
        "border": f"2px solid {error_color}",
    }
    error_text = f"IK Error: {ik_error_mm:.3f} mm  |  ||FK(IK(X_cmd)) - X_cmd||"

    # Info panel
    info = html.Div([
        html.P(f"Baseline EE: [{baseline_ee[0]:.4f}, {baseline_ee[1]:.4f}, {baseline_ee[2]:.4f}]"),
        html.P(f"Target EE:   [{X_cmd[0]:.4f}, {X_cmd[1]:.4f}, {X_cmd[2]:.4f}]"),
        html.P(f"IK EE:       [{ik_ee[0]:.4f}, {ik_ee[1]:.4f}, {ik_ee[2]:.4f}]"),
        html.Hr(style={"borderColor": "#333"}),
        html.P(f"Baseline joints: {np.array2string(theta_baseline, precision=1)}"),
        html.P(f"IK joints:       {np.array2string(theta_ik, precision=1)}"),
        html.P(f"Joint diff:      {np.array2string(theta_ik - theta_baseline, precision=1)}"),
    ])

    return fig, error_text, error_style, info


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    import socket
    # Get local IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except:
        local_ip = "127.0.0.1"

    print("\n" + "="*60)
    print("  SO101 Interactive IK Visualizer")
    print(f"  Local:   http://127.0.0.1:8050")
    print(f"  Network: http://{local_ip}:8050")
    print("="*60 + "\n")
    app.run(debug=True, host="0.0.0.0", port=8050)
