#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
turtle_drawer —— 让小海龟画出指定数字（一笔画曲线最终版）
================================================
任务：让小海龟在画布上画出自己学号的最后两位数字
日志格式：[INFO] [TurtleDrawer]: Drawing digit 7, step 1...
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen

# ==========================================================
# 数字笔画定义（局部坐标）
# 网格：x 从 0(左) 到 3(右)，y 从 0(下) 到 4(上)
# 数据格式：{数字: [ [笔尖移动序列], ... ]}
#   直线段： (x, y, 是否落笔)
#   曲线段： {'center': (x, y), 'radius': R, 'sweep': 弧度, 'start_angle': 起始角}
# ==========================================================
DIGITS = {
    '0': [[(0, 0, False), (3, 0, True), (3, 4, True), (0, 4, True), (0, 0, True)]],
    '1': [[(1.5, 0, False), (1.5, 4, True)],
          [(0.7, 0, False), (2.3, 0, True)]],
    '2': [[(0, 4, False), (3, 4, True), (0, 0, True), (3, 0, True)]],
    '3': [[(0, 4, False), (3, 4, True), (1.0, 2, True), (3, 2, True)],
          [(3, 2, False), (3, 0, True), (0, 0, True)]],
    '4': [[(2.0, 4, False), (2.0, 2, True), (3, 2, True)],
          [(3, 2, False), (3, 0, True)]],
    '5': [[(3, 4, False), (0, 4, True), (0, 2, True), (3, 2, True), (3, 0, True), (0, 0, True)]],
    '6': [[(3, 4, False), (0, 4, True), (0, 0, True), (3, 0, True), (3, 2, True), (0, 2, True)]],
    '7': [[(0, 4, False), (3, 4, True)],
          [(3, 4, False), (1.0, 0, True)]],
    # ⭐ 核心优化：上下两圆在腰部相切，起点/终点都在腰部，方向均为向左
    '8': [[
        {'center': (1.5, 2.9), 'radius': 0.9, 'sweep': -2 * math.pi, 'start_angle': -math.pi / 2},
        {'center': (1.5, 1.0), 'radius': 1.0, 'sweep': 2 * math.pi, 'start_angle': math.pi / 2}
    ]],
    '9': [[(3, 2, False), (3, 4, True), (0, 4, True), (0, 2, True), (3, 2, True)],
          [(3, 2, False), (3, 0, True), (0.5, 0, True)]],
}

# 每个数字的颜色（RGB 0-255）
DIGIT_COLORS = {
    '0': (255, 60, 60),
    '1': (255, 160, 0),
    '2': (255, 230, 0),
    '3': (60, 230, 60),
    '4': (0, 230, 220),
    '5': (60, 140, 255),
    '6': (170, 90, 255),
    '7': (255, 90, 200),
    '8': (255, 200, 60),
    '9': (160, 255, 160),
}

# 画布参数
CANVAS_W = 11.0888
DIGIT_W = 3.0
DIGIT_H = 4.0
GAP = 1.5
BOTTOM = 3.0

LINEAR_SPEED = 2.5
TOLERANCE = 0.08
TICK = 0.02
MAX_SECONDS = 25.0


class TurtleDrawer(Node):
    """让小海龟按笔画绘制数字的节点"""

    def __init__(self, digits='07'):
        super().__init__('turtle_drawer')
        self.digits = digits
        self.cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.teleport_cli = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute',
            callback_group=self.cb_group)
        self.pen_cli = self.create_client(
            SetPen, '/turtle1/set_pen',
            callback_group=self.cb_group)

        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 teleport_absolute 服务...')
        while not self.pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_pen 服务...')

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0

    def set_pen(self, r, g, b, width=3, off=0):
        """设置画笔颜色与开关。off=1 抬笔，off=0 落笔"""
        req = SetPen.Request()
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.width = int(width)
        req.off = int(off)
        future = self.pen_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def teleport(self, x, y, theta):
        """瞬移到绝对位置（抬笔移动到笔画起点）"""
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        future = self.teleport_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.cur_x, self.cur_y, self.cur_theta = float(x), float(y), float(theta)

    def stop(self):
        self.pub.publish(Twist())

    def go_to(self, goal_x, goal_y, label=''):
        """朝目标点走：边转边走，到达后停下"""
        start = time.time()
        while True:
            dx = goal_x - self.cur_x
            dy = goal_y - self.cur_y
            dist = math.hypot(dx, dy)
            if dist < TOLERANCE:
                break
            if time.time() - start > MAX_SECONDS:
                self.get_logger().warn(f'单笔超时，强制结束: {label}')
                break

            target_theta = math.atan2(dy, dx)
            d_theta = math.atan2(
                math.sin(target_theta - self.cur_theta),
                math.cos(target_theta - self.cur_theta))

            twist = Twist()
            twist.angular.z = max(-4.0, min(4.0, d_theta * 4.0))
            twist.linear.x = 0.0 if abs(d_theta) > 0.6 else min(LINEAR_SPEED, dist * 3.0)
            self.pub.publish(twist)

            self.cur_theta += twist.angular.z * TICK
            self.cur_x += twist.linear.x * math.cos(self.cur_theta) * TICK
            self.cur_y += twist.linear.x * math.sin(self.cur_theta) * TICK
            time.sleep(TICK)

        self.stop()
        time.sleep(0.08)

    # ⭐ 核心升级：新增 is_continuous 参数，支持不抬笔连续画弧
    def arc(self, cx, cy, radius, total_sweep, start_angle=0.0, is_continuous=False, label='', r=255, g=200, b=60):
        """让海龟沿圆弧行走（支持自定义起点和方向，支持一笔连续画）"""
        # 根据起始角度计算起点坐标
        start_x = cx + radius * math.cos(start_angle)
        start_y = cy + radius * math.sin(start_angle)

        # 根据顺/逆时针方向计算初始朝向（切线方向）
        if total_sweep > 0:      # 逆时针
            theta = start_angle + math.pi / 2.0
            omega = LINEAR_SPEED / radius
        else:                    # 顺时针
            theta = start_angle - math.pi / 2.0
            omega = -LINEAR_SPEED / radius

        if not is_continuous:
            # 抬笔 -> 瞬移到起点 -> 落笔
            self.set_pen(r, g, b, width=3, off=1)
            self.teleport(start_x, start_y, theta)
            self.set_pen(r, g, b, width=3, off=0)
        else:
            # 连续绘制时，海龟已经在起点且朝向正确，只需更新内部坐标状态
            self.cur_x = start_x
            self.cur_y = start_y
            self.cur_theta = theta

        v = LINEAR_SPEED
        # 按固定周期计算需要走多少步
        ticks = max(1, int(round(abs(total_sweep) / abs(omega) / TICK)))

        self.get_logger().info(
            f'  {label}: 走圆弧 圆心=({cx:.2f}, {cy:.2f}) 半径={radius:.2f} '
            f'扫过={math.degrees(total_sweep):.1f}° 方向={"逆时针" if total_sweep > 0 else "顺时针"} '
            f'({ticks} 个控制周期)')

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega

        for _ in range(ticks):
            self.pub.publish(twist)
            self.cur_theta += omega * TICK
            self.cur_x += v * math.cos(self.cur_theta) * TICK
            self.cur_y += v * math.sin(self.cur_theta) * TICK
            time.sleep(TICK)

        self.stop()

    def to_canvas(self, lx, ly, offset_x):
        """把局部坐标 (0..3, 0..4) 映射到画布坐标"""
        return (offset_x + lx, BOTTOM + ly)

    def draw_digit(self, ch, offset_x, step_counter):
        """画一个数字，返回更新后的 step 计数"""
        r, g, b = DIGIT_COLORS.get(ch, (255, 255, 255))
        strokes = DIGITS[ch]

        for stroke_idx, stroke in enumerate(strokes, start=1):
            step_counter += 1
            has_curve = any(isinstance(s, dict) for s in stroke)

            if has_curve:
                n_points = len([s for s in stroke if not isinstance(s, dict)])
                self.get_logger().info(
                    f'Drawing digit {ch}, step {step_counter}... '
                    f'(笔画 {stroke_idx} / {len(strokes)}, {n_points} 个画线段)')
            else:
                n_points = len([p for p in stroke if p[2]])
                self.get_logger().info(
                    f'Drawing digit {ch}, step {step_counter}... '
                    f'(笔画 {stroke_idx} / {len(strokes)}, {n_points} 个画线段)')

            # 记录当前笔画是否已经开始绘制（用于判断曲线是否连续）
            stroke_started = False

            # 如果是纯直线，先抬笔瞬移到起点
            if not has_curve:
                self.set_pen(r, g, b, width=3, off=1)
                sx, sy = self.to_canvas(stroke[0][0], stroke[0][1], offset_x)
                if len(stroke) > 1:
                    nx, ny = self.to_canvas(stroke[1][0], stroke[1][1], offset_x)
                    theta = math.atan2(ny - sy, nx - sx)
                else:
                    theta = 0.0
                self.teleport(sx, sy, theta)
                self.set_pen(r, g, b, width=3, off=0)
                stroke_started = True

            # 依次走完本笔的所有线段或曲线
            seg_no = 0
            for seg in stroke:
                if isinstance(seg, dict):
                    # 曲线段
                    cx, cy = self.to_canvas(seg['center'][0], seg['center'][1], offset_x)
                    radius = seg['radius'] * (DIGIT_W / 3.0)
                    sweep = seg['sweep']
                    start_angle = seg.get('start_angle', 0.0)

                    self.get_logger().info(
                        f'  digit {ch} step {step_counter}: '
                        f'curve → 圆心({cx:.2f}, {cy:.2f}) 半径 {radius:.2f}')

                    # 关键：如果本笔画已经开始了（即有前一段曲线），则连续绘制，不抬笔
                    is_cont = stroke_started

                    self.arc(cx, cy, radius, sweep, start_angle, is_continuous=is_cont,
                             label=f'digit {ch} step {step_counter}', r=r, g=g, b=b)
                    stroke_started = True
                else:
                    # 直线段
                    lx, ly, pen_down = seg
                    if not pen_down:
                        continue
                    seg_no += 1
                    tx, ty = self.to_canvas(lx, ly, offset_x)
                    self.get_logger().info(
                        f'  digit {ch} step {step_counter}: '
                        f'segment {seg_no}/{n_points} → ({tx:.2f}, {ty:.2f})')
                    self.go_to(tx, ty, label=f'digit {ch} step {step_counter}')
                    stroke_started = True

            # 抬笔（off=1）
            self.set_pen(r, g, b, width=3, off=1)

        return step_counter

    def run(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'TurtleDrawer 启动，准备绘制数字: {self.digits}')
        self.get_logger().info('=' * 55)

        n = len(self.digits)
        total_w = n * DIGIT_W + (n - 1) * GAP
        start_x = (CANVAS_W - total_w) / 2.0

        step_counter = 0
        for i, ch in enumerate(self.digits):
            if ch not in DIGITS:
                self.get_logger().warn(f'跳过不支持的字符: {ch}')
                continue
            offset_x = start_x + i * (DIGIT_W + GAP)
            self.get_logger().info(
                f'>>> 开始绘制第 {i + 1} 个数字: {ch}  '
                f'(起点 x={offset_x:.2f}, 颜色 RGB={DIGIT_COLORS[ch]})')
            step_counter = self.draw_digit(ch, offset_x, step_counter)
            self.get_logger().info(f'<<< 数字 {ch} 绘制完成')

        self.set_pen(255, 255, 255, width=3, off=1)
        self.teleport(CANVAS_W / 2.0, CANVAS_W / 2.0, 0.0)
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'全部完成！共 {step_counter} 步，数字: {self.digits}')
        self.get_logger().info('=' * 55)


def main(args=None):
    rclpy.init(args=args)

    digits = '07'
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    if argv:
        digits = argv[-1]

    node = TurtleDrawer(digits=digits)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('被用户中断')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
