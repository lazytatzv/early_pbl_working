import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
from gait_core import *

vx = 10.0
vy = 0.0
vtheta = 0.0

stride_x = vx * (STEP_LENGTH / STRIDE_X_DIV)
dist_per_cycle = stride_x / (1.0 - SWING_RATIO)

def calculate_global_pos(total_phase, leg_id):
    phase = total_phase % 1.0
    body_g_x = total_phase * dist_per_cycle
    body_g_y = 0.0

    local_x, local_y, local_z, is_swing, body_offset_x, body_offset_y = calculate_leg_pos(phase, leg_id, vx, vy, vtheta)

    foot_g_x = body_g_x + local_x + body_offset_x
    foot_g_y = body_g_y + local_y + body_offset_y
    foot_g_z = local_z

    actual_body_g_x = body_g_x + body_offset_x
    actual_body_g_y = body_g_y + body_offset_y

    return foot_g_x, foot_g_y, foot_g_z, is_swing, actual_body_g_x, actual_body_g_y

cycles = 3.0
frames = 150
total_phases = np.linspace(0, cycles, frames, endpoint=False)

sim_data = []
for tp in total_phases:
    leg_positions = []
    swings = []
    body_x, body_y = 0.0, 0.0
    for i in range(4):
        x, y, z, is_swing, bx, by = calculate_global_pos(tp, i)
        leg_positions.append((x, y, z))
        swings.append(is_swing)
        body_x, body_y = bx, by
    sim_data.append((tp, leg_positions, swings, body_x, body_y))

fig, ax = plt.subplots(figsize=(12, 8))

def update_plot(frame_idx, target_ax):
    target_ax.clear()
    target_ax.set_title("Global Path & Footprints (3 Walking Cycles)", fontsize=14, weight='bold')
    target_ax.set_xlim(-120, 550)
    target_ax.set_ylim(-180, 180)
    target_ax.set_aspect('equal')
    target_ax.set_xlabel("Global X (Forward) [mm]", fontsize=12)
    target_ax.set_ylabel("Global Y (Right/Left) [mm]", fontsize=12)
    target_ax.grid(True, linestyle='--', alpha=0.6)

    cog_path_x = [d[3] for d in sim_data[:frame_idx+1]]
    cog_path_y = [d[4] for d in sim_data[:frame_idx+1]]
    target_ax.plot(cog_path_x, cog_path_y, 'r-', linewidth=2, label="Body Center Path (with Shift)")

    tp, leg_pos, swings, bx, by = sim_data[frame_idx]
    body_outline = np.array([
        [FR_X, FR_Y], [FL_X, FL_Y], [BL_X, BL_Y], [BR_X, BR_Y], [FR_X, FR_Y]
    ])
    body_outline_g_x = bx + body_outline[:, 0]
    body_outline_g_y = by + body_outline[:, 1]
    target_ax.plot(body_outline_g_x, body_outline_g_y, 'k-', linewidth=2, label="Robot Body")
    target_ax.plot(bx, by, 'ko', markersize=8)

    for i in range(4):
        sx = bx + offsets[i][0]
        sy = by + offsets[i][1]
        fx, fy, _ = leg_pos[i]
        target_ax.plot([sx, fx], [sy, fy], 'k-', alpha=0.4)

    stance_points = []
    for i in range(4):
        if not swings[i]:
            stance_points.append((leg_pos[i][0], leg_pos[i][1]))
    if len(stance_points) >= 3:
        pts = np.array(stance_points)
        center = np.mean(pts, axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        sort_idx = np.argsort(angles)
        pts_sorted = pts[sort_idx]
        polygon = plt.Polygon(pts_sorted, closed=True, fill=True, color='green', alpha=0.15)
        target_ax.add_patch(polygon)

    for i in range(4):
        foot_path_x = []
        foot_path_y = []
        for f in range(frame_idx + 1):
            f_tp, f_leg_pos, f_swings, _, _ = sim_data[f]
            if not f_swings[i]:
                foot_path_x.append(f_leg_pos[i][0])
                foot_path_y.append(f_leg_pos[i][1])
        
        if foot_path_x:
            color = ['orange', 'cyan', 'magenta', 'lightgreen'][i]
            target_ax.scatter(foot_path_x, foot_path_y, color=color, s=20, alpha=0.5, edgecolors='none')
            curr_x, curr_y, _ = leg_pos[i]
            curr_color = 'red' if swings[i] else 'green'
            target_ax.plot(curr_x, curr_y, marker='o', color=curr_color, markersize=10, markeredgecolor='black')
            target_ax.text(curr_x+3, curr_y+3, f"{leg_names[i]}", fontsize=9, weight='bold')

    target_ax.plot([], [], 'go', markersize=8, label="Stance Foot (On Ground)")
    target_ax.plot([], [], 'ro', markersize=8, label="Swing Foot (In Air)")
    target_ax.legend(loc="upper left")

fig_static, ax_st = plt.subplots(figsize=(12, 6))
update_plot(frames - 1, ax_st)
for idx in [25, 75, 125]:
    _, _, _, bx, by = sim_data[idx]
    body_outline = np.array([[FR_X, FR_Y], [FL_X, FL_Y], [BL_X, BL_Y], [BR_X, BR_Y], [FR_X, FR_Y]])
    ax_st.plot(bx + body_outline[:, 0], by + body_outline[:, 1], 'k--', alpha=0.3)
    ax_st.text(bx, by+10, f"Cycle {idx//50 + 1}", fontsize=8, alpha=0.6, ha='center')

plt.tight_layout()

script_dir = os.path.dirname(os.path.abspath(__file__))
output_png = os.path.join(script_dir, "..", "sim_output", "gait_global_simulation.png")
output_gif = os.path.join(script_dir, "..", "sim_output", "gait_global_simulation.gif")
output_mp4 = os.path.join(script_dir, "..", "sim_output", "gait_global_simulation.mp4")

plt.savefig(output_png, dpi=150)
plt.close(fig_static)
print(f"Saved global trajectory plot to {output_png}")

def animate(idx):
    update_plot(idx, ax)

ani = animation.FuncAnimation(fig, animate, frames=frames, interval=330)
ani.save(output_gif, writer='pillow', fps=3)
print(f"Saved global simulation animation to {output_gif}")

try:
    ani.save(output_mp4, writer='ffmpeg', fps=3, bitrate=1800)
    print(f"Saved global simulation video to {output_mp4}")
except Exception as e:
    print(f"Could not save global MP4 video: {e}")

plt.close(fig)
