# turtle_drawer — 小海龟绘制学号数字

> ROS 2 功能包 ｜ 选题 A：ROS 2 小海龟"数字"走位

## 一、功能说明

控制 `turtlesim` 小海龟在画布上**绘制指定数字**，用于画出学号的最后两位。

- 支持 **0–9 全部数字**（笔画化定义，任意两位组合都能画）
- 每个数字使用**不同颜色**（可配置）
- 终端**实时输出日志**，格式为：`Drawing digit 7, step 1...`
- 笔画之间**自动抬笔**，不会拖出多余的线

## 二、运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 24.04.5 LTS (Noble) |
| ROS 2 | **Jazzy Jalisco** |
| 依赖包 | `rclpy`、`geometry_msgs`、`turtlesim` |

## 三、编译

```bash
# 1. 把 turtle_drawer 放进工作空间
mkdir -p ~/ros2_ws/src
cp -r turtle_drawer ~/ros2_ws/src/

# 2. 编译
cd ~/ros2_ws
colcon build --packages-select turtle_drawer

# 3. 加载环境
source ~/ros2_ws/install/setup.bash
```

## 四、运行

**需要两个终端。**

**终端 1 —— 启动小海龟图形界面：**
```bash
ros2 run turtlesim turtlesim_node
```

**终端 2 —— 启动绘制节点：**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtle_drawer turtle_drawer 07      # ← 参数换成你的学号后两位
```

**参数说明**：命令行最后一个参数是**要绘制的数字**，默认 `07`。

## 五、终端输出示例

```
[INFO] [turtle_drawer]: =======================================================
[INFO] [turtle_drawer]: TurtleDrawer 启动，准备绘制数字: 07
[INFO] [turtle_drawer]: =======================================================
[INFO] [turtle_drawer]: >>> 开始绘制第 1 个数字: 0  (起点 x=1.79, 颜色 RGB=(255, 60, 60))
[INFO] [turtle_drawer]: Drawing digit 0, step 1... (笔画 1 / 1, 4 个画线段)
[INFO] [turtle_drawer]:   digit 0 step 1: segment 1/4 → (1.79, 3.00)
[INFO] [turtle_drawer]:   digit 0 step 1: segment 2/4 → (4.79, 3.00)
[INFO] [turtle_drawer]:   digit 0 step 1: segment 3/4 → (4.79, 7.00)
[INFO] [turtle_drawer]:   digit 0 step 1: segment 4/4 → (1.79, 7.00)
[INFO] [turtle_drawer]: <<< 数字 0 绘制完成
[INFO] [turtle_drawer]: >>> 开始绘制第 2 个数字: 7  (起点 x=6.29, 颜色 RGB=(255, 90, 200))
[INFO] [turtle_drawer]: Drawing digit 7, step 2... (笔画 1 / 2, 1 个画线段)
...
[INFO] [turtle_drawer]: 全部完成！共 3 步，数字: 07
```

## 六、实现原理

### 6.1 用到的三个 ROS 2 接口

| 功能 | 接口 | 类型 |
|---|---|---|
| 控制移动 | 发布 `geometry_msgs/Twist` 到 `/turtle1/cmd_vel` | 话题 |
| 控制画笔开关/颜色 | 调用 `/turtle1/set_pen` | 服务 |
| 瞬移到笔画起点 | 调用 `/turtle1/teleport_absolute` | 服务 |

### 6.2 核心逻辑：抬笔画线

```
抬笔 → 瞬移到笔画起点 → 落笔 → 移动到线段终点（留下线条）
                            ↓
                     抬笔 → 瞬移到下一笔起点 → ...
```

**为什么需要抬笔**：数字由多个不相连的笔画组成（例如 `7` = 横线 + 斜线）。
从一笔跳到下一笔起点时若不抬笔，移动过程会拖出多余的线，图案就糊了。

### 6.3 ⭐ 关键设计：服务调用必须等完成

```python
# 移动用「发布话题」—— 不用等回复，海龟持续接收速度指令
self.pub.publish(twist)

# 改画笔用「调用服务」—— 必须等它执行完成！
future = self.pen_cli.call_async(req)
rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
```

**原因**：如果"抬笔"请求还没处理完就发出"移动"指令，海龟会**带着笔移动**，
在屏幕上拖出一条多余的线。这是本任务最容易出现的 bug。

### 6.4 数字的坐标编码

数字定义在 **3×4 的局部网格**中（`x`: 0→3 从左到右，`y`: 0→4 从下到上）：

```python
'7': [[(0, 4, False), (3, 4, True)],      # 第一笔：顶部横线
      [(3, 4, False), (1.0, 0, True)]],   # 第二笔：斜线
#      (x, y, 是否落笔)
#      False = 抬笔跳到该点（不留线）
#      True  = 从上一个点画线到该点
```

绘制时通过 `to_canvas()` 把局部坐标映射到画布坐标：

```python
def to_canvas(self, lx, ly, offset_x):
    return (offset_x + lx, BOTTOM + ly)
```

### 6.5 移动控制

采用**比例控制（P 控制）**：

1. 计算当前位置到目标点的距离和方向
2. 方向偏差大时**只转不走**（避免画出斜线）
3. 偏差小时**边转边走**，速度与距离成正比（距离越近越慢，避免冲过头）
4. 到达容差范围（`TOLERANCE = 0.08`）后停止

## 七、可调参数

全部在 `turtle_drawer/turtle_drawer.py` 文件顶部：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `DIGIT_COLORS` | 各数字不同颜色 | 每个数字的 RGB 颜色（0–255）|
| `DIGIT_W` / `DIGIT_H` | 3.0 / 4.0 | 单个数字的宽高（局部坐标单位）|
| `GAP` | 1.5 | 两个数字之间的间距 |
| `BOTTOM` | 3.0 | 数字距画布底部的距离 |
| `LINEAR_SPEED` | 2.5 | 画线时的线速度 |
| `TOLERANCE` | 0.08 | 到达目标的判定阈值 |
| `MAX_SECONDS` | 25.0 | 单笔超时保护（秒）|

**改完需重新编译**：
```bash
cd ~/ros2_ws && colcon build --packages-select turtle_drawer
source ~/ros2_ws/install/setup.bash
```

## 八、目录结构

```
turtle_drawer/                       ← 功能包根目录
├── package.xml                      ← 包描述：名称、版本、依赖
├── setup.py                         ← 安装配置：注册可执行入口点
├── setup.cfg                        ← 安装脚本路径配置
├── resource/
│   └── turtle_drawer                ← ament 资源标记文件（空）
└── turtle_drawer/                   ← Python 模块目录
    ├── __init__.py                  ← 模块标记（空）
    └── turtle_drawer.py             ← 节点主程序
```

## 九、常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `No executable found` | `setup.py` 的 `entry_points` 没注册 | 检查后重新 `colcon build` |
| `Package 'turtle_drawer' not found` | 没加载工作空间环境 | `source ~/ros2_ws/install/setup.bash` |
| 卡在 `Waiting for service ...` | 小海龟节点没启动 | 先在另一个终端跑 `ros2 run turtlesim turtlesim_node` |
| 海龟不动 | 之前的残留速度指令 | 重启 `turtlesim_node` |
| 笔画间有多余连线 | 抬笔时序错乱 | 已通过 `spin_until_future_complete` 处理；若仍有，加大 `timeout_sec` |
| 数字画歪 | 速度太快 | 把 `LINEAR_SPEED` 调小到 1.5 |
| 海龟跑出画布 | 坐标超范围 | 画布范围是 `0 ~ 11.0888`，检查 `DIGIT_W`/`GAP`/`BOTTOM` |
