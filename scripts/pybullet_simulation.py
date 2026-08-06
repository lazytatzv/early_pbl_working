import os
import sys
import pybullet
import numpy as np
import yaml
import imageio

# gait_core の読み込み
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import gait_core

# 逆運動学 (IK) 関数の定義
def solve_ik(leg_id, px, py, pz):
    dx, dy = gait_core.offsets[leg_id]
    lx = px - dx
    ly = py - dy

    # 1. Hip IK (Yaw)
    current_angle = np.arctan2(ly, lx)
    diff_angle = current_angle - gait_core.base_angles[leg_id]
    
    # 角度正規化 (-pi 〜 pi)
    diff_angle = (diff_angle + np.pi) % (2.0 * np.pi) - np.pi

    # 2. Knee IK (Pitch)
    target_z = abs(pz)
    sin_knee = target_z / gait_core.L2
    sin_knee = np.clip(sin_knee, 0.0, 1.0)
    theta_knee = np.arcsin(sin_knee) # rad

    return diff_angle, theta_knee

def main():
    # 物理シミュレーションの初期化 (DIRECTモード: ヘッドレス画面なし実行)
    pybullet.connect(pybullet.DIRECT)
    pybullet.setGravity(0, 0, -9.81)

    # 床（コリジョンとビジュアルを持つ薄いボックス）の生成
    floor_col = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[10, 10, 0.01])
    floor_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[10, 10, 0.01], rgbaColor=[0.8, 0.8, 0.8, 1.0])
    pybullet.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_col, baseVisualShapeIndex=floor_visual, basePosition=[0, 0, -0.01])

    # ロボット URDF のロード (少し浮かせた状態から落とす)
    urdf_path = "onshape_robot/robot.urdf"
    if not os.path.exists(urdf_path):
        print(f"Error: URDF not found at {urdf_path}")
        return
        
    robot_id = pybullet.loadURDF(urdf_path, basePosition=[0, 0, 0.13], baseOrientation=[0, 0, 0, 1], useFixedBase=True)

    # ジョイント名のインデックスへのマッピング
    joint_name_to_index = {}
    print("----- PyBullet Joint Info -----")
    for i in range(pybullet.getNumJoints(robot_id)):
        info = pybullet.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        joint_type = info[2] # 0: revolute, 1: prismatic, 2: spherical, 3: planar, 4: fixed
        print(f"Index {i}: {joint_name} (Type: {joint_type})")
        joint_name_to_index[joint_name] = i
        # デフォルトのモーター制御力を無効化（位置制御を有効にするため）
        pybullet.setJointMotorControl2(robot_id, i, pybullet.VELOCITY_CONTROL, force=0)
    print("-------------------------------")

    # 各脚の可動ジョイント名 (URDFの構成に一致)
    leg_joints = [
        # Leg 0 (FR)
        ("leg_assembly_v20_leg1_top_2_v6_fixed", "leg_assembly_v20_leg2_center_v8_fixed"),
        # Leg 1 (BL)
        ("leg_assembly_v20__1_leg1_top_2_v6_3_fixed", "leg_assembly_v20__1_leg2_center_v8_2_fixed"),
        # Leg 2 (FL)
        ("leg_assembly_v20__2_leg1_top_2_v6_5_fixed", "leg_assembly_v20__2_leg2_center_v8_3_fixed"),
        # Leg 3 (BR)
        ("leg_assembly_v20__3_leg1_top_2_v6_7_fixed", "leg_assembly_v20__3_leg2_center_v8_4_fixed"),
    ]

    # シミュレーションループ設定
    gif_frames = []
    fps = 30
    duration = 4.0  # 4秒間
    steps = int(duration * 240) # PyBulletはデフォルト240Hz
    
    # 進行速度コマンド（前進）
    vx, vy, vtheta = 1.0, 0.0, 0.0
    
    # カメラ設定
    camera_distance = 0.5
    camera_yaw = 55
    camera_pitch = -25

    print("Running PyBullet physical simulation loop...")

    for step in range(steps):
        # 物理時間の経過 (240Hz)
        t_sec = step / 240.0
        # 歩行サイクル位相の計算
        phase = (t_sec * 1000.0 % gait_core.CYCLE_TIME) / gait_core.CYCLE_TIME

        # 各脚の関節角度を計算して適用
        for i in range(4):
            tx, ty, tz, is_swing, _, _ = gait_core.calculate_leg_pos(phase, i, vx, vy, vtheta)
            
            # 逆運動学を解いてジョイント角を算出
            hip_angle, knee_angle = solve_ik(i, tx, ty, tz)

            hip_joint_name, knee_joint_name = leg_joints[i]
            
            if hip_joint_name in joint_name_to_index and knee_joint_name in joint_name_to_index:
                hip_idx = joint_name_to_index[hip_joint_name]
                knee_idx = joint_name_to_index[knee_joint_name]

                # Hip関節: ヨー軸（Z軸）周り
                pybullet.setJointMotorControl2(robot_id, hip_idx, pybullet.POSITION_CONTROL, targetPosition=hip_angle, force=1.2)
                
                # Knee関節: ピッチ軸周り。初期アライメント（垂直=90度/1.57rad）からの差分制御
                target_knee_pos = 1.57 - knee_angle
                pybullet.setJointMotorControl2(robot_id, knee_idx, pybullet.POSITION_CONTROL, targetPosition=target_knee_pos, force=1.2)
                if step < 5 and i == 0:
                    print(f"Step {step} Leg 0: hip_angle={hip_angle:.4f}, knee_angle={knee_angle:.4f}, target_knee_pos={target_knee_pos:.4f}")

        # 物理演算を1ステップ進める
        pybullet.stepSimulation()

        # 30FPSごとにカメラ画像をレンダリング
        if step % (240 // fps) == 0:
            # ロボットの位置にカメラターゲットを追従させる
            pos, _ = pybullet.getBasePositionAndOrientation(robot_id)
            camera_target = [pos[0], pos[1], pos[2]]
            
            view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target,
                distance=camera_distance,
                yaw=camera_yaw,
                pitch=camera_pitch,
                roll=0,
                upAxisIndex=2
            )
            proj_matrix = pybullet.computeProjectionMatrixFOV(
                fov=60,
                aspect=16.0/9.0,
                nearVal=0.1,
                farVal=10.0
            )
            (_, _, rbgColor, _, _) = pybullet.getCameraImage(
                width=640,
                height=360,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=pybullet.ER_TINY_RENDERER
            )
            # RGBカラー情報の抽出
            rgb_array = np.reshape(rbgColor, (360, 640, 4))[:, :, :3]
            gif_frames.append(rgb_array)

    pybullet.disconnect()

    # MP4動画の保存
    output_dir = "sim_output"
    os.makedirs(output_dir, exist_ok=True)
    mp4_path = os.path.join(output_dir, "pybullet_simulation.mp4")
    
    print(f"Saving simulation animation to {mp4_path}...")
    imageio.mimsave(mp4_path, gif_frames, fps=fps, format="FFMPEG", codec="h264")
    print("PyBullet physical simulation finish successfully!")

if __name__ == "__main__":
    main()
