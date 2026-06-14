# -*- coding: utf-8 -*-

# config.py

"""
vel, rotate_vel為自走車PID數值, 可於arduino程式碼查看

於ros_receive_and_data_processing/AI_node.py使用

前左、前右、後左、後右
"""
speed_ratio = 50
vel = 18.0*speed_ratio              # 900 RPM forward
vel_slow = 7.0*speed_ratio          # 350 RPM forward
vel_crawl = 6.0*speed_ratio         # 300 RPM crawl
# Unity RPM notes (speed_ratio=50 → RPM = coefficient × 50):
#   Forward all-wheel: needs ~600+ to overcome friction.
#   In-place rotation: 250 RPM stalled when sent continuously; 200–240 RPM
#   works with pulsing (see yaw_alignment_change_report.md). HW4 used 250 slow.
rotate_vel = 6.0*speed_ratio        # 300 RPM — AMCL nav turns
rotate_vel_slow = 5.0*speed_ratio   # 250 RPM — visual align/drive (use with pulse)
rotate_vel_median = 5.5*speed_ratio   # 275 RPM — coarse align bursts
ACTION_MAPPINGS = {
    "FORWARD": [vel, vel, vel, vel],  # 前進
    "FORWARD_SLOW": [vel_slow, vel_slow, vel_slow, vel_slow],  # 前進
    "FORWARD_CRAWL": [vel_crawl, vel_crawl, vel_crawl, vel_crawl],  # 慢速前進 (grasp)
    "LEFT_FRONT": [rotate_vel, rotate_vel * 1.2, rotate_vel, rotate_vel * 1.2],  # 左前
    "COUNTERCLOCKWISE_ROTATION": [
        -rotate_vel,
        rotate_vel,
        -rotate_vel,
        rotate_vel,
    ],  # 左自轉
    "COUNTERCLOCKWISE_ROTATION_SLOW": [
        -rotate_vel_slow,
        rotate_vel_slow,
        -rotate_vel_slow,
        rotate_vel_slow,
    ],  # 慢左自轉
    "COUNTERCLOCKWISE_ROTATION_MEDIAN": [
        -rotate_vel_median,
        rotate_vel_median,
        -rotate_vel_median,
        rotate_vel_median,
    ],  # 中速左自轉
    "BACKWARD": [-vel, -vel, -vel, -vel],  # 後退
    "BACKWARD_SLOW": [-vel_slow, -vel_slow, -vel_slow, -vel_slow],  # 後退
    "CLOCKWISE_ROTATION": [rotate_vel, -rotate_vel, rotate_vel, -rotate_vel],  # 右自轉
    "CLOCKWISE_ROTATION_SLOW": [
        rotate_vel_slow,
        -rotate_vel_slow,
        rotate_vel_slow,
        -rotate_vel_slow,
    ],  # 右慢自轉
    "CLOCKWISE_ROTATION_MEDIAN": [
        rotate_vel_median,
        -rotate_vel_median,
        rotate_vel_median,
        -rotate_vel_median,
    ],  # 中右自轉
    "RIGHT_FRONT": [rotate_vel * 1.2, rotate_vel, rotate_vel * 1.2, rotate_vel],  # 右前
    "RIGHT_SHIFT": [rotate_vel, -rotate_vel, -rotate_vel, rotate_vel],
    "LEFT_SHIFT": [-rotate_vel, rotate_vel, rotate_vel, -rotate_vel],
    "STOP": [0.0, 0.0, 0.0, 0.0],
}
