import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
from gait_core import *

vx = 10.0
vy = 0.0
vtheta = 0.0

fig = plt.figure(figsize=(12, 6))
ax2d = fig.add_subplot(121)
ax3d = fig.add_subplot(122, projection='3d')

cycles = 1.0
frames = 100
phases = np.linspace(0, cycles, frames, endpoint=False)

sim_data = []
for phase in phases:
    leg_positions = []
    swings = []
    body_x, body_y = 0.0, 0.0
    for i in range(4):
        x, y, z, is_swing, bx, by = calculate_leg_pos(phase, i, vx, vy, vtheta)
        leg_positions.append((x, y, z))
        swings.append(is_swing)
        body_x, body_y = bx, by
    sim_data.append((phase, leg_positions, swings, body_x, body_y))

def update_plot(frame_idx, ax_2d, ax_3d):
    ax_2d.clear()
    ax_3d.clear()

    phase, leg_pos, swings, bx, by = sim_data[frame_idx]

    ax_2d.set_title(f"Support Polygon & Center of Gravity (Phase: {phase:.2f})")
    ax_2d.set_xlim(-180, 180)
    ax_2d.set_ylim(-180, 180)
    ax_2d.set_aspect('equal')
    ax_2d.set_xlabel("X (Forward) [mm]")
    ax_2d.set_ylabel("Y (Right) [mm]")
    ax_2d.grid(True)

    body_outline = np.array([
        [FR_X, FR_Y], [FL_X, FL_Y], [BL_X, BL_Y], [BR_X, BR_Y], [FR_X, FR_Y]
    ])
    body_outline[:, 0] += bx
    body_outline[:, 1] += by
    ax_2d.plot(body_outline[:, 0], body_outline[:, 1], 'k--', alpha=0.5, label="Body Frame")
    ax_2d.plot(bx, by, 'ko', markersize=10, label="CoG (Body Center)")

    stance_points = []
    for i in range(4):
        x, y, z = leg_pos[i]
        color = 'ro' if swings[i] else 'go'
        label = f"{leg_names[i]} (Swing)" if swings[i] else f"{leg_names[i]} (Stance)"
        ax_2d.plot(x, y, color, markersize=8)
        ax_2d.text(x+2, y+2, leg_names[i], fontsize=10, weight='bold')
        if not swings[i]:
            stance_points.append((x, y))

    if len(stance_points) >= 3:
        pts = np.array(stance_points)
        center = np.mean(pts, axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        sort_idx = np.argsort(angles)
        pts_sorted = pts[sort_idx]
        polygon = plt.Polygon(pts_sorted, closed=True, fill=True, color='green', alpha=0.15, label="Support Polygon")
        ax_2d.add_patch(polygon)
        
        if len(pts_sorted) == 3:
            p_cog = np.array([bx, by])
            p0, p1, p2 = pts_sorted[0], pts_sorted[1], pts_sorted[2]
            
            def sign(p1, p2, p3):
                return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
            
            d1 = sign(p_cog, p0, p1)
            d2 = sign(p_cog, p1, p2)
            d3 = sign(p_cog, p2, p0)
            
            has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
            has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
            
            stable = not (has_neg and has_pos)
            stability_str = "STABLE" if stable else "UNSTABLE"
            stability_color = "green" if stable else "red"
            ax_2d.text(-170, 150, f"Stability: {stability_str}", color=stability_color, fontsize=12, weight='bold')

    ax_2d.legend(loc="lower left")

    ax_3d.set_title("3D Foot Trajectory")
    ax_3d.set_xlim(-180, 180)
    ax_3d.set_ylim(-180, 180)
    ax_3d.set_zlim(-160, 20)
    ax_3d.set_xlabel("X (Forward)")
    ax_3d.set_ylabel("Y (Right)")
    ax_3d.set_zlabel("Z (Height)")

    for i in range(4):
        sx, sy = offsets[i]
        sx += bx
        sy += by
        sz = 0.0
        
        fx, fy, fz = leg_pos[i]
        
        theta_current = np.arctan2(fy - sy, fx - sx)
        
        kx = sx + L1 * np.cos(theta_current)
        ky = sy + L1 * np.sin(theta_current)
        kz = 0.0
        
        ax_3d.plot([sx, kx], [sy, ky], [sz, kz], 'k-', linewidth=3, alpha=0.7)
        ax_3d.plot([kx, fx], [ky, fy], [kz, fz], 'b-', linewidth=3)
        ax_3d.plot([sx], [sy], [sz], 'ko', markersize=5)
        ax_3d.plot([fx], [fy], [fz], 'ro' if swings[i] else 'go', markersize=6)
        
    body_x_pts = [FR_X+bx, FL_X+bx, BL_X+bx, BR_X+bx, FR_X+bx]
    body_y_pts = [FR_Y+by, FL_Y+by, BL_Y+by, BR_Y+by, FR_Y+by]
    body_z_pts = [0, 0, 0, 0, 0]
    ax_3d.plot(body_x_pts, body_y_pts, body_z_pts, 'k-', linewidth=2)

fig_static = plt.figure(figsize=(15, 10))
ax1 = fig_static.add_subplot(221)
ax2 = fig_static.add_subplot(222, projection='3d')
update_plot(12, ax1, ax2)
ax3 = fig_static.add_subplot(223)
ax4 = fig_static.add_subplot(224, projection='3d')
update_plot(62, ax3, ax4)

plt.tight_layout()

script_dir = os.path.dirname(os.path.abspath(__file__))
output_png = os.path.join(script_dir, "..", "sim_output", "gait_simulation.png")
output_gif = os.path.join(script_dir, "..", "sim_output", "gait_simulation.gif")
output_mp4 = os.path.join(script_dir, "..", "sim_output", "gait_simulation.mp4")

plt.savefig(output_png, dpi=150)
plt.close(fig_static)
print(f"Saved summary static plot to {output_png}")

def animate(idx):
    update_plot(idx, ax2d, ax3d)

ani = animation.FuncAnimation(fig, animate, frames=frames, interval=330)
ani.save(output_gif, writer='pillow', fps=3)
print(f"Saved walk simulation animation to {output_gif}")

try:
    ani.save(output_mp4, writer='ffmpeg', fps=3, bitrate=1800)
    print(f"Saved walk simulation video to {output_mp4}")
except Exception as e:
    print(f"Could not save MP4 video: {e}")

plt.close(fig)

