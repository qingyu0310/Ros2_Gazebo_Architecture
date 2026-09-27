# 轮腿机器人 LQR 平衡教程

> 目标：让 `try/` 里的 `wheel_leg_robot`（两个驱动轮 + 两条腿）在平地上靠 LQR 自己站住，
> 并且能按 `/chassis/cmd_vel` 前后走、左右转。
>
> 路线：模型参数 → A/B 矩阵 → K → 状态量从哪来 → 底盘代码 → 参数文件 → launch → 验收。
>
> 这篇文档只讲**两个轮子上的平衡**。腿的伸缩、跳跃、滚转调平不在这里（`lqr/` 那边的
> `docs/伸缩腿关节规划.md`、`docs/滚转平衡与跳跃规划.md` 讲那些）。

---

## 0. 现在有什么、缺什么

写代码之前先把这条链上的环节点一遍。平衡这条链一共五环，**前四环已经在了，缺的只有控制律**：

| 环 | 在哪 | 状态 |
|---|---|---|
| 仿真世界与物理 | `gazebo/worlds/empty.sdf`（物理步长 1 ms） | 有 |
| 模型（关节、质量、摩擦、传感器、力矩入口） | `project/robot/models/`、`gazebo/plugins/` | 有 |
| gz ↔ ROS 的桥 | `project/launch/sim.launch.py` | 有 |
| 求解器（DARE 迭代求 K） | `framework/algorithm/controller/lqr.hpp` | 有 |
| 执行端（目标力矩 → 关节力矩） | `ros2_layer/node/motor/motor.hpp` | 有 |
| 关节/姿态反馈 | `joint_encoder` 宏、`imu` 宏 | 有 |
| **控制律（状态 → 力矩）** | — | **缺，就是本篇要写的** |

所以这篇不是从零造轮子，是**把已经有的东西接起来、再写中间那一段**。参考实现是
`lqr/src/project/node/lqr/lqr.cpp`（那边整车能站住），本文每一步都对着它讲，但**参数、
话题、关节名全都要按 `try/` 的模型重新算**，不能照抄。

---

## 1. 参数：模型需要给人什么

### 1.1 降阶成什么模型

两条腿现在是 **fixed**（标准姿态 α = 40°，见 `modules/leg.xacro`），整机就是一个刚体
架在两个同轴驱动轮上 —— 教科书里的 **cart-pole（小车倒立摆）**：

- 「小车」＝ 两个轮子（它们只会滚，不会倒）
- 「摆」＝ 车上除轮子以外的一切（底盘 + 胯 + 大腿 + 膝 + 小腿）

状态取四个：

```
x = [ p   ,  ṗ    ,  θ   ,  θ̇   ]
      轮子等效位移  前进速度  车身俯仰角  俯仰角速度
      (m)         (m/s)     (rad)      (rad/s)
```

控制只有一个：**两个轮子的公共力矩** τ（左右各给 τ，俯仰环只吃这个公共分量；
方向差的那一份留给偏航，见 §5.5）。所以是 4 状态 1 输入。

### 1.2 五个物理参数

| 符号 | 含义 | 本文的值 | 从哪来 |
|---|---|---|---|
| `r` | 轮子半径 | 0.04000 m | `modules/wheel.xacro` 的 `radius` |
| `M` | **等效**小车质量 | 0.637500 kg | 两轮的质量 + 各自转动惯量折合到平动 |
| `m` | 车身质量（除轮子外） | 1.250000 kg | URDF 里除轮子外所有 link 的质量和 |
| `l` | 车身质心到**轮轴**的距离 | 0.068249 m | URDF 里除轮子外的质量加权质心 |
| `I` | 车身绕质心、绕 **y 轴** 的俯仰惯量 | 0.002591 kg·m² | 各 link 的 `iyy` + 平行轴项 |
| `b` | 等效粘性阻尼 | 0.02 | 估的量，见下 |

`b` 是滚阻/粘滞阻尼，只进 A 矩阵的速度项。它**不是**需要精确测的量：取 0 这套系统照样
能解出能用的 K，取 0.01~0.05 只影响速度项的阻尼感。这里跟 `lqr` 那台取同一个值 0.02。

### 1.3 每个量怎么从 URDF 量出来

**`r`、`m_w`、`I_w`** 直接读 `modules/wheel.xacro`。有一处必须注意：

> 轮子绕**轴**（关节轴 = y）的转动惯量，是 macro 里的 **`izz` = 0.00027**，
> 不是 `iyy` = 0.00016。因为轮子的几何/惯性系写了 `rpy="1.57079632679 0 0"`（绕 x 转 90°
> 把圆柱放倒），惯性系的 z 轴映射到 link 的 y 轴 —— 也就是轮轴。取错了 K 会偏。

**`M` 为什么不是两轮质量之和**：电机力矩有一部分花在把轮子自己转起来上。把这份惯量折成
平动质量，方程里就不用再单独写轮子的转动方程：

```
M = Σ轮子 ( m_w + I_w / r² ) = 2 × (0.15 + 0.00027 / 0.04²) = 2 × 0.31875 = 0.637500 kg
```

**`m`、`l`**：把除轮子外所有 link 按质量加权。`base_link` 原点就在**轮轴高度**上
（见 `bodys/wheel_leg_robot.xacro` 的头注释），所以 `l` 就是合成质心的 z 坐标。

**`I`**：逐个 link 算 `Iyy_i + m_i·(dx_i² + dz_i²)`，`d` 是该 link 质心相对**车身合成质心**的偏移。

> **`dx²` 这一项最容易漏。** 大腿在 x = +21.4 mm、小腿在 x = −21.4 mm、膝圆柱在 x = +42.9 mm，
> 都离质心不近；只算 `dz²` 会得到 0.002525，比正确的 0.002591 小 2.5%。量级不大，但既然
> 是脚本算的，就别手算。

#### 计算脚本

在 `try/` 目录下跑这段，出来的就是上表那一列（本文的数就是它跑出来的，不是手抄的）：

```bash
# 把 $(find project) / $(find gazebo) 换成本地路径（xacro 在源码目录里展不开，
# 因为 install 空间那份才是完整的；这里用临时目录，不动你的工作区）
set -e
rm -rf /tmp/wlr_param && mkdir -p /tmp/wlr_param
cp -r project gazebo /tmp/wlr_param/
grep -rl '\$(find ' /tmp/wlr_param --include=*.xacro \
  | xargs sed -i 's|\$(find project)|/tmp/wlr_param/project|g; s|\$(find gazebo)|/tmp/wlr_param/gazebo|g'
xacro /tmp/wlr_param/project/robot/models/bodys/wheel_leg_robot.xacro > /tmp/wlr_param/robot.urdf

python3 - <<'PY'
import xml.etree.ElementTree as ET
import numpy as np

root = ET.parse('/tmp/wlr_param/robot.urdf').getroot()
R = 0.04

# 关节链：本模型所有关节 rpy 都是 0，往 base_link 下累加 xyz 就是各 link 原点的位置
J = {}
for j in root.findall('joint'):
    o = j.find('origin')
    J[j.find('child').get('link')] = (
        j.find('parent').get('link'),
        np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3))

def origin_pos(link, acc=np.zeros(3)):
    if link not in J:
        return acc
    parent, xyz = J[link]
    return origin_pos(parent, acc + xyz)

links = {}
for l in root.findall('link'):
    n = l.get('name')
    m = float(l.find('inertial/mass').get('value'))
    io = l.find('inertial/origin')
    off = np.array([float(v) for v in io.get('xyz').split()]) if io is not None else np.zeros(3)
    I = l.find('inertial/inertia')
    links[n] = dict(m=m, pos=origin_pos(n) + off,
                    I_link=np.array([float(I.get(k)) for k in ('ixx', 'iyy', 'izz')]))

wheels = [n for n in links if 'wheel' in n]
body   = [n for n in links if 'wheel' not in n]

# 轮子：绕轴的惯量取 izz 分量（几何绕 x 转 90°，惯性系 z 轴 = link 的 y 轴 = 轮轴）
M = sum(links[n]['m'] + links[n]['I_link'][2] / R**2 for n in wheels)

m   = sum(links[n]['m'] for n in body)
com = sum(links[n]['m'] * links[n]['pos'] for n in body) / m
I   = sum(links[n]['I_link'][1] + links[n]['m'] * ((links[n]['pos'] - com)[[0, 2]] ** 2).sum()
        for n in body)

print(f"r = {R:.5f}\nM = {M:.6f} kg\nm = {m:.6f} kg\nl = {com[2]:.6f} m\nI = {I:.6f} kg·m^2")
print(f"总质量 = {sum(v['m'] for v in links.values()):.4f} kg，车身质心 x = {com[0]:+.6f} m")
PY
```

跑出来：

```
r = 0.04000
M = 0.637500 kg
m = 1.250000 kg
l = 0.068249 m
I = 0.002591 kg·m^2
总质量 = 1.5500 kg，车身质心 x = +0.002402 m
```

`I` 的合成明细（脚本里加一行 `print` 就能看到）：

| link | 质量 kg | 质心 z m | `Iyy` 自身 | `m·(dx²+dz²)` |
|---|---|---|---|---|
| `base_link` | 1.1100 | +0.07199 | 0.00230000 | 0.00002196 |
| `*_hip_link` ×2 | 0.0050 | +0.07199 | 0.00000030 | 0.00000010 |
| `*_thigh_link` ×2 | 0.0300 | +0.05399 | 0.00000850 | 0.00001698 |
| `*_knee_link` ×2 | 0.0050 | +0.03600 | 0.00000030 | 0.00001340 |
| `*_calf_link` ×2 | 0.0300 | +0.01800 | 0.00000850 | 0.00008664 |
| | | | **Σ = 0.002591** | |

#### 和 `lqr/` 那台的对照

同一套脚本套在 `lqr` 那台的 URDF 上，得到 M 0.6375 / m 1.25 / l 0.06825 / I 0.002591，
和 `lqr/src/project/params/control.yaml` 里留下的四个值**逐位一致**。这不是巧合：
`try/` 的零件尺寸本来就是照着 `lqr` 那台抄的（轮子 40mm、腿 56mm/40°、底盘 110×90×30）。
对上了说明这套量法没错，也说明 `try/` 这台可以直接用那组参数起步。

### 1.4 什么时候这些参数就失效了

`l` 和 `I` 都是「车身质心在哪」的函数。**腿一伸缩，质心就动了** ——
`modules/leg.xacro` 顶上那段写了伸缩的实现方式（`fixed` 换 `revolute`），一旦解开：

- `l` 会随高度在 ~0.056~0.068 m 之间变，`I` 也跟着变；
- A、B 矩阵整体变，**这一篇算出来的 K 立刻不适用**。

届时要走两条路之一：把高度分档、每档离线算一组 K 然后查表切换；或者按高度在线重算
A/B 再解一次（`Lqr::configure()` + `solve()` 本来就能反复调）。**不要在腿动起来之后
还用这一组 K**，表现是高度变了之后就站不住。

### 1.5 模型自检

- 开环特征值：`+13.5466 / −13.5556 / −0.0106 / 0`（连续时间）—— 有一个**正实根**，
  **开环不稳定**，这正是需要闭环的原因，不是模型错了。
- 那一对 `−0.0106 / 0` 来自「**车身竖直、小车匀速走**」这个平衡流形：无阻尼时它是临界
  的，`b = 0.02` 把它压成微稳定。**它就是要靠 `q[0]`（位移）那一项来钉住的**，所以 Q 里
  位移权重不能给 0。
- 离散化后（dt = 1 ms）开环极点最大模 1.0135 > 1，同样说明不稳定。
- 平衡的物理条件 `l > 0` 成立（质心在轮轴上方 68 mm），如果哪天算出 `l < 0`，那是 URDF
  的质心算错了，先修模型再谈控制。

---

## 2. 从参数到 K

### 2.1 连续时间方程

记 `p = I·(M + m) + M·m·l²`（量纲 kg²·m²，别约掉），则

```
        ⎡ 0      1                0                 0    ⎤
A_c  =  ⎢ 0   -(I+ml²)b/p      -m²gl²/p             0    ⎥
        ⎢ 0      0                0                 1    ⎥
        ⎣ 0    mlb/p          mgl(M+m)/p            0    ⎦

        ⎡                                    0                                    ⎤
B_c  =  ⎢  2·[ (I+ml²)/r + m·l ] / p                                              ⎥
        ⎢                                    0                                    ⎥
        ⎣ -2·[ m·l/r + M + m ] / p                                                 ⎦
```

这两块和 `lqr.cpp:130-141` 逐字一致（那边的变量名是 `continuous_a` / `continuous_b`，
`body_pitch_about_com` 就是 `I + m·l²`）。

**B 里面那个 2 是约定，不是笔误。** 它来自「`u` 是**每个轮子**的力矩，两个轮子同时给」：
合力 = 2τ/r。如果你改用「`u` = 两个轮子的力矩总和」，就要把 2 去掉、同时把
`max_effort` 翻倍。两种写法都自洽，**混用则是灾难**：K 会整体差一倍，表现为「增益怎么调
都不对」。本文和 `lqr.cpp` 都用「每轮力矩」这一种。

### 2.2 离散化

LQR 在离散域求解。1 ms 的步长下，前向欧拉足够：

```cpp
discrete_a = Identity() + continuous_a * dt;     // dt = 0.001
discrete_b = continuous_b * dt;
```

真正决定 K 的是 `dt` 与**实际控制周期**是否一致。定时器跑成 2 ms 而 `dt` 写 0.001，
控制器会比设计的更激进（等效把 B 放大了一倍）。所以 §8.3 里有一条「用
`ros2 topic hz /motor/left/command` 核一遍实际周期」。

### 2.3 求解

用现成的 `framework/algorithm/controller/lqr.hpp`：

```cpp
Controller controller_;
controller_.configure(discrete_a, discrete_b, q, r);   // q/r 见 §6.1
controller_.solve(100000, 1e-12);                       // 迭代黎卡提，收敛即返回
const auto& K = controller_.gain();
// 控制律： u = -K (x - x_target)
```

`solve()` 内部就是 `P ← Q + Aᵀ P (A − B K)` 反复迭代到 P 不再变（`lqr.hpp:77-101`）。
`1e-12` 的收敛判据下要迭代约 5 万次，在构造函数里跑一次、几毫秒的事，**每拍不要重解**。

### 2.4 本文这组参数解出来的 K

按 §1.2 的参数 + §6.1 的 Q/R：

```
K = [-0.045481, -0.193570, -2.254953, -1.083929]
    ↑位移       ↑速度       ↑俯仰      ↑俯仰角速度
```

节点启动时会打这一行（`RCLCPP_INFO`），**拿它跟上面这组对一下**：

- 数量级对不上 → 参数（尤其 `r`、`l`）传错了；
- 顺序反了（最大的是速度项） → Q 的四个数写错位了；
- 4 个数变成 3 个或 5 个 → `q` 数组长度不对，`configure_lqr()` 里该抛异常。

**绝对值最大的是俯仰那一项**，这是对的：平衡环的主要矛盾就是俯仰，位移和速度只负责
「别越走越远」。

### 2.5 符号必须对一遍

`K(0,2) = −2.255 < 0`，所以 `u = −K(0,2)·θ = +2.255·θ`：**θ > 0 时给正力矩**。

现在核对 θ > 0 到底是不是「往前倾」：

- 俯仰角是绕 **+y** 的旋转。绕 +y 转正角度时，天向（+z）被带向 **+x**（前方）——
  所以 **θ > 0 = 车身往前倾**。
- 轮子关节轴是 `0 1 0`（`modules/wheel.xacro`），绕 +y 正转就是**往前滚**（+x）。

合起来：往前倾 → 两个轮子往前滚 → 把轮子送到车身下面 → 把车拽回来。**物理上正确**。

如果实测是反的（一发力就往一边加速倒下，而且越倒越猛），说明 IMU 的俯仰符号或者力矩
符号跟这里相反，一个参数就能翻回来：

- `pitch_sign`：θ 的符号（`1.0` 或 `-1.0`）
- `control_sign`：最终力矩的符号（`1.0` 或 `-1.0`）

**两个都留着，别只留一个。** 有时候是「IMU 装反了」，有时候是「关节轴方向和我以为的
不一样」，两种情况的修法不是同一个参数。

---

## 3. 状态量从哪来

| 状态 | 话题 | 类型 | 取哪个字段 | 换算 |
|---|---|---|---|---|
| `p` 位移 | `/joint_states` | `sensor_msgs/JointState` | 两个轮关节的 `position` | `r·(pos_L + pos_R)/2` |
| `ṗ` 速度 | `/joint_states` | 同上 | 两个轮关节的 `velocity` | `r·(vel_L + vel_R)/2` |
| `θ` 俯仰 | `/imu` | `sensor_msgs/Imu` | `orientation` 四元数 | `asin(clamp(2(wy − zx)))` |
| `θ̇` 俯仰角速度 | `/imu` | 同上 | `angular_velocity.y` | 直接用 |
| 偏航角速度 | `/imu` | 同上 | `angular_velocity.z` | 直接用（只给转向环） |

### 3.1 轮子状态：`/joint_states`

关节名是 `left_wheel_joint` / `right_wheel_joint`（`bodys/wheel_leg_robot.xacro` 里
`<xacro:wheel>` 的 `joint_name`）。三个坑：

1. **必须两个轮子都在这一帧里**才更新，只报一个就用上一帧。`lqr.cpp:191-206` 里那段
   越界检查 + `wheel_count != 2` 就是这个意思；本文的版本把 name/position/velocity
   三个数组**一起截短**再遍历，比逐个判越界干净。
2. **`position` 是累积角，不是「相对原点的位移」。** `type="continuous"` 的关节，gz 报的
   是转过的总角度，一直涨。所以必须自己记一个 `position_origin_`，用差值当状态；
   每次重新标定原点（§5.7 第三件）就是把这个基准更新掉。忘了这一步，车一开始就会
   疯狂往一个方向冲（它以为自己在离原点 100 米的地方）。
3. **这一路是里程计，会漂。** 轮子打滑的时候，轮子转过的角不等于车走的距离。好在
   LQR 给位移的权重最小，漂一点不影响站住，只影响「停在哪个位置」—— 往前走的时候
   表现为速度指令和实际速度有个小比例误差。真实车上要修这个得再加一路视觉/激光；
   仿真里 `mu1 = 1.0` 打滑很少，不用管。

### 3.2 姿态：`/imu`

`gazebo/plugins/sensor/imu.xacro` 挂的是 gz 的 Imu 传感器，默认在 `base_link` 上、
1000 Hz、话题 `/imu`。**在仿真里它的 `orientation` 是位姿真值**（gz 直接算出来的，不是
陀螺积分），所以不漂、可以直接用 —— 用四元数转俯仰角，不碰加速度计：

```cpp
const double w = msg.orientation.w, x = msg.orientation.x;
const double y = msg.orientation.y, z = msg.orientation.z;
// 绕 y 的俯仰角：四元数 → ZYX 欧拉角的 pitch 分量。clamp 是防浮点越界让 asin 出 NaN
const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
pitch_      = pitch_sign_ * std::asin(sin_pitch);
pitch_rate_ = pitch_sign_ * msg.angular_velocity.y;
yaw_rate_   = msg.angular_velocity.z;
```

同一个公式在 `framework/algorithm/filter/attitude/quaternion_attitude.hpp` 的
`euler_rpy_rad()` 里也有（`sinp = 2(wy − zx)`），两边是自洽的。

IMU 的位置在 `bodys/wheel_leg_robot.xacro` 里是 `0 0 0.092`（胯轴上方 20 mm，底盘上方
35 mm）。**位置不影响俯仰角**，只影响角速度里叠加的平动耦合（可以忽略）。

### 3.3 为什么直接订 `/imu`，不用 imu 节点的 `/imu/euler_rad`

`try/` 里有一个 imu 节点（`ros2_layer/node/imu/imu.hpp`），把原始 IMU 解算成
`/imu/euler_rad` 等话题。平衡环也可以订 `/imu/euler_rad` 的 `vector.y`，但**本文不用**：

- 那个节点的姿态是**滤波器**（陀螺积分 + 加速度修正，`accel_correction_gain = 0.1`），
  实车上这是必需的（gz 的真值在实车上没有），但它比真值**多一个延迟**。平衡环是
  最快的那个环，不该为了统一而吃这份延迟。
- 少一跳少一处对齐：控制环直接吃传感器，出问题时不必判断是「滤波器的问题」还是
  「控制器的问题」。
- 和 `lqr.cpp` 一致（那边也是直接订 `/imu`），两边能互相对照。

**实车移植时改这一处**：真实 IMU 的 orientation 是积分出来的、会漂，那时才该订
imu 节点的融合输出，并且 `pitch_` 的来源换掉、其余不动。

---

## 4. 底盘代码怎么写

### 4.1 放在哪（这里有一个需要你拍板的分叉）

现在 `project/node/chassis/` 里是**四轮那套运动学**（`Twist` → 四个轮速），
对轮腿车没有意义：两个轮子的车不可能靠左右轮速差做滑移转向，而且底盘指令的含义变了
—— 不再直接给轮速，而是给「速度目标」，由平衡环解算成力矩。

两条路：

| 方案 | 做法 | 代价 |
|---|---|---|
| **A（推荐）** | 就地改写 `project/node/chassis/`，`ChassisNode` 变成平衡控制器 | 四轮那套运动学删掉 |
| B | 新增 `project/node/balance/`，跟 chassis 并存 | 两个节点都能往电机发力矩，得手动保证只起一个 |

**我按 A 写。** 理由：同一时刻只能有一套「谁在往电机发力矩」的逻辑，两个节点并存时
这个约束只能靠「记得别两个都启动」维持，迟早出事；而四轮运动学对轮腿车的未来没用
（以后就算要转向也是靠偏航环，不是那套运动学）。**如果你打算保留四轮车那条线**，
就说一声，改成 B 只是把文件挪个地方、CMake 里加两行。

按 A 的好处是：`chassis.cpp` 的入口几乎不用改，`params/chassis.yaml` 的段名不用改，
CMakeLists 不用改，只有 `chassis.hpp` 一个文件重写。

### 4.2 类骨架

```cpp
/**
 * @file chassis.hpp
 * @brief 底盘节点：轮腿车的平衡控制器 —— 轮子状态 + 车身姿态 -> 两个轮子的力矩
 *
 * 分工：求解器在 framework（algorithm::controller::Lqr），单轮的执行端在 ros2_layer
 *       （MotorNode），本节点只干「把状态喂给 LQR、把力矩分给两个轮子」这一层。
 *
 * 一个进程三个节点：chassis + motor_left + motor_right，由同一个 executor 一起 spin
 *   （子节点不加进执行器，它们的订阅和定时器一个都不会跑）
 * 话题：
 *   订 /chassis/cmd_vel      geometry_msgs/Twist，只读 linear.x 和 angular.z
 *   订 /imu                  sensor_msgs/Imu，要 orientation 和 angular_velocity
 *   订 /joint_states         sensor_msgs/JointState，要两个轮关节的 position/velocity
 *   发 /motor/<左|右>/command std_msgs/Float64，力矩 N·m（effort 模式）
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/controller/lqr.hpp"
#include "ros2_layer/node/motor/motor.hpp"

class ChassisNode : public rclcpp::Node
{
public:
    // 4 状态 1 输入：x = [位移, 速度, 俯仰, 俯仰角速度]，u = 单轮力矩
    using Controller  = algorithm::controller::Lqr<4, 1, double>;
    using State       = Controller::State;
    using StateMatrix = Controller::StateMatrix;
    using InputMatrix = Controller::InputMatrix;

    // 轮子下标。数组全按这个顺序对齐，别再散着写
    static constexpr std::size_t kLeft       = 0;
    static constexpr std::size_t kRight      = 1;
    static constexpr std::size_t kWheelCount = 2;

    explicit ChassisNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// 本节点和两个电机子节点，交给同一个 executor 一起 spin
    std::vector<rclcpp::Node::SharedPtr> nodes();

private:
    static constexpr const char* kMotorNodeNames[kWheelCount] = {"motor_left", "motor_right"};

    Controller controller_;          // 求解器。构造时算一次 K，之后每拍只调 update()

    // ---- 模型参数（由 URDF 量出来）----
    double wheel_radius_       {0.040};     // r
    double cart_mass_          {0.6375};    // M
    double body_mass_          {1.25};      // m
    double body_com_height_    {0.06825};   // l
    double body_pitch_inertia_ {0.002591};  // I
    double cart_damping_       {0.02};      // b
    double gravity_            {9.81};

    // ---- 控制与安全 ----
    double dt_                 {0.001};
    double max_effort_         {0.25};
    double fall_angle_rad_     {0.70};
    double pitch_sign_         {1.0};
    double control_sign_       {1.0};

    // ---- 遥控 ----
    std::string cmd_vel_topic_         {"/chassis/cmd_vel"};
    std::string imu_topic_             {"/imu"};
    std::string joint_states_topic_    {"/joint_states"};
    std::string control_mode_          {"effort"};
    double command_timeout_            {0.30};
    double max_velocity_               {0.20};
    double max_yaw_rate_               {1.0};
    double yaw_kp_                     {0.01};
    double max_differential_           {0.02};
    double yaw_sign_                   {1.0};

    // ---- 接线（底盘给两个电机的）----
    std::array<std::string, kWheelCount> joint_names_{};
    std::array<std::string, kWheelCount> command_topics_{};
    std::array<std::string, kWheelCount> force_topics_{};
    std::array<MotorNode::SharedPtr, kWheelCount> motor_{};

    // ---- 状态：最近一帧反馈 + 控制用的四个量 ----
    double pitch_              {0.0};   // θ
    double pitch_rate_         {0.0};   // θ̇
    double yaw_rate_           {0.0};   // 偏航角速度，只给转向环
    double wheel_position_     {0.0};   // r·(pos_L+pos_R)/2，累积角的里程
    double wheel_velocity_     {0.0};   // ṗ
    double position_origin_    {0.0};   // 位移原点的标定值
    double position_target_    {0.0};   // 位置目标，由速度指令积分而来
    double target_velocity_    {0.0};
    double target_yaw_rate_    {0.0};

    bool has_imu_              {false};
    bool has_joint_state_      {false};
    bool origin_initialized_   {false};
    bool fallen_               {false};
    std::chrono::steady_clock::time_point last_command_time_ {std::chrono::steady_clock::now()};

    // ---- ROS 句柄 ----
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_sub_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kWheelCount> command_pub_{};

    void   configure_lqr();
    void   on_imu(const sensor_msgs::msg::Imu& msg);
    void   on_joint_state(const sensor_msgs::msg::JointState& msg);
    void   on_cmd_vel(const geometry_msgs::msg::Twist& msg);
    void   update();
    void   publish_effort(double left_effort, double right_effort);
    double command_age_s() const;
};
```

`on_cmd_vel` 和 `command_age_s()` 和四轮那版一模一样（`chassis.hpp:208-230`），搬过来即可；
`on_cmd_vel` 里要把速度指令夹到 `±max_velocity_`、偏航指令夹到 `±max_yaw_rate_`
（`lqr.cpp:165-170` 的做法），并且**用 `std::isfinite` 挡掉 NaN** —— 遥控发来一个 NaN
会让整个状态炸成 NaN，`std::clamp(NaN, ...)` 是不保证挡得住的。

需要额外 `#include <array>`（上面骨架里的 `std::array` 用到了）。

### 4.3 构造函数：参数 + 求解 + 接线

```cpp
ChassisNode::ChassisNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("chassis", options)
{
    // ---- 模型参数：由 URDF 量出来，见 docs/轮腿机器人LQR平衡教程.md §1.3 ----
    wheel_radius_       = declare_parameter("wheel_radius",        0.040);
    cart_mass_          = declare_parameter("cart_mass",           0.6375);   // M
    body_mass_          = declare_parameter("body_mass",           1.25);     // m
    body_com_height_    = declare_parameter("body_com_height",     0.06825);  // l
    body_pitch_inertia_ = declare_parameter("body_pitch_inertia",  0.002591); // I
    cart_damping_       = declare_parameter("cart_damping",        0.02);     // b
    gravity_            = declare_parameter("gravity",             9.81);

    // ---- 控制 ----
    dt_                 = declare_parameter("dt",                  0.001);
    max_effort_         = std::abs(declare_parameter("max_effort",        0.25));
    fall_angle_rad_     = std::abs(declare_parameter("fall_angle_rad",    0.70));
    pitch_sign_         = declare_parameter("pitch_sign",  1.0);
    control_sign_       = declare_parameter("control_sign", 1.0);

    // ---- 遥控：linear.x 前后速度（正=前进），angular.z 偏航角速度（正=左转）----
    cmd_vel_topic_      = declare_parameter("cmd_vel_topic",       "/chassis/cmd_vel");
    max_velocity_       = std::abs(declare_parameter("max_velocity",      0.20));
    max_yaw_rate_       = std::abs(declare_parameter("max_yaw_rate",      1.0));
    yaw_kp_             = std::abs(declare_parameter("yaw_kp",            0.01));
    max_differential_   = std::abs(declare_parameter("max_differential",  0.02));
    yaw_sign_           = declare_parameter("yaw_sign", 1.0);
    command_timeout_    = std::abs(declare_parameter("command_timeout_s", 0.30));

    // ---- 反馈话题 ----
    imu_topic_          = declare_parameter("imu_topic",           "/imu");
    joint_states_topic_ = declare_parameter("joint_states_topic",  "/joint_states");

    // 力矩模式是这套控制的前提：平衡环要的是「给我这个力矩」，
    // 中间插一层速度环等于多一个积分器 + 一层延迟，两个环会互相顶
    control_mode_ = declare_parameter("control_mode", "effort");
    if (control_mode_ != "effort")
    {
        throw std::invalid_argument("平衡环必须用 effort 模式：control_mode 只能是 'effort'");
    }
    if (wheel_radius_ <= 0.0 || dt_ <= 0.0)
    {
        throw std::invalid_argument("wheel_radius 和 dt 必须大于 0");
    }

    configure_lqr();   // 一次性求解，见 §4.4

    // ---- 两个电机的接线。哪个关节、力矩发到哪个话题，由底盘统一给 ----
    joint_names_    = {declare_parameter("left_joint_name",  "left_wheel_joint"),
                       declare_parameter("right_joint_name", "right_wheel_joint")};
    command_topics_ = {declare_parameter("left_command_topic",  "/motor/left/command"),
                       declare_parameter("right_command_topic", "/motor/right/command")};
    force_topics_   = {declare_parameter("left_force_topic",  "/motor/left/cmd_force"),
                       declare_parameter("right_force_topic", "/motor/right/cmd_force")};

    for (std::size_t i = 0; i < kWheelCount; ++i)
    {
        rclcpp::NodeOptions motor_options;
        motor_options.parameter_overrides({rclcpp::Parameter("joint_name",         joint_names_[i]),
                                          rclcpp::Parameter("command_topic",       command_topics_[i]),
                                          rclcpp::Parameter("force_topic",         force_topics_[i]),
                                          rclcpp::Parameter("joint_states_topic",  joint_states_topic_),
                                          rclcpp::Parameter("control_mode",        control_mode_)});
        motor_[i]       = std::make_shared<MotorNode>(kMotorNodeNames[i], motor_options);
        command_pub_[i] = create_publisher<std_msgs::msg::Float64>(command_topics_[i], 10);
    }

    // 回调只缓存最新一帧，控制律统一放在定时器里算，跟话题频率解耦
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(*msg); });

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]() { update(); });
}
```

### 4.4 `configure_lqr()`：参数 → K

```cpp
void ChassisNode::configure_lqr()
{
    const double m = body_mass_;
    const double M = cart_mass_;
    const double l = body_com_height_;
    const double I = body_pitch_inertia_;
    const double b = cart_damping_;
    const double p = I * (M + m) + M * m * l * l;   // 共同分母，别约

    StateMatrix continuous_a = StateMatrix::Zero();
    continuous_a(0, 1) = 1.0;
    continuous_a(1, 1) = -(I + m * l * l) * b / p;
    continuous_a(1, 2) = -m * m * gravity_ * l * l / p;
    continuous_a(2, 3) = 1.0;
    continuous_a(3, 1) = m * l * b / p;
    continuous_a(3, 2) = m * gravity_ * l * (M + m) / p;

    InputMatrix continuous_b = InputMatrix::Zero();
    const double body_pitch_about_com = I + m * l * l;
    continuous_b(1, 0) =  2.0 * (body_pitch_about_com / wheel_radius_ + m * l) / p;
    continuous_b(3, 0) = -2.0 * (m * l / wheel_radius_ + M + m) / p;

    // 前向欧拉离散化：dt = 1 ms 时误差可忽略
    const StateMatrix discrete_a = StateMatrix::Identity() + continuous_a * dt_;
    const InputMatrix discrete_b = continuous_b * dt_;

    // Q 的四个数按状态顺序给：[位移, 速度, 俯仰, 俯仰角速度]
    const auto q_values = declare_parameter<std::vector<double>>("q", {1.0, 1.0, 1000.0, 550.0});
    if (q_values.size() != static_cast<std::size_t>(State::RowsAtCompileTime))
    {
        throw std::invalid_argument("参数 q 必须正好 4 个数（位移/速度/俯仰/俯仰角速度）");
    }
    StateMatrix q = StateMatrix::Zero();
    for (std::size_t i = 0; i < q_values.size(); ++i)
    {
        q(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) = q_values[i];
    }

    Controller::ControlMatrix r;
    r(0, 0) = declare_parameter("r", 1.0);

    controller_.configure(discrete_a, discrete_b, q, r);
    controller_.solve(100000, 1e-12);

    const auto& k = controller_.gain();
    RCLCPP_INFO(get_logger(), "LQR K = [%.6f, %.6f, %.6f, %.6f]",
                k(0, 0), k(0, 1), k(0, 2), k(0, 3));
}
```

### 4.5 两个反馈回调

```cpp
void ChassisNode::on_imu(const sensor_msgs::msg::Imu& msg)
{
    const double w = msg.orientation.w, x = msg.orientation.x;
    const double y = msg.orientation.y, z = msg.orientation.z;

    // 绕 +y 的俯仰角（ZYX 欧拉角的 pitch 分量）。clamp 防浮点越界让 asin 出 NaN
    const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    pitch_      = pitch_sign_ * std::asin(sin_pitch);
    pitch_rate_ = pitch_sign_ * msg.angular_velocity.y;
    yaw_rate_   = msg.angular_velocity.z;
    has_imu_    = true;
}

void ChassisNode::on_joint_state(const sensor_msgs::msg::JointState& msg)
{
    // 三个数组一起截短，别各自判越界
    const std::size_t count = std::min({msg.name.size(), msg.position.size(), msg.velocity.size()});

    double      position_sum = 0.0;
    double      velocity_sum = 0.0;
    std::size_t wheel_count  = 0;

    for (std::size_t i = 0; i < count; ++i)
    {
        if (msg.name[i] != joint_names_[kLeft] && msg.name[i] != joint_names_[kRight])
        {
            continue;
        }
        position_sum += msg.position[i];
        velocity_sum += msg.velocity[i];
        ++wheel_count;
    }

    // 两个轮子必须都在这一帧里才更新：只报一个说明这一帧不完整，
    // 宁可沿用上一帧，也别拿「半个轮子的状态」当整车状态
    if (wheel_count != kWheelCount)
    {
        return;
    }

    // 取两个轮子的平均：一起往前走时各贡献一半；差的那一半是偏航，不算进前进位移
    // position 是累积角（continuous 关节），所以这只是「相对原点的里程」，原点在 update() 里标定
    wheel_position_ = wheel_radius_ * position_sum * 0.5;
    wheel_velocity_ = wheel_radius_ * velocity_sum * 0.5;
    has_joint_state_ = true;
}
```

### 4.6 控制节拍 `update()`

```cpp
void ChassisNode::update()
{
    // 反馈没齐（IMU 或关节状态一帧都没到）就不发力
    if (!has_imu_ || !has_joint_state_)
    {
        publish_effort(0.0, 0.0);
        return;
    }

    // 姿态超限 = 已经倒了。归零不是「保护电机」，是「别躺着还使劲蹬地」
    if (std::abs(pitch_) > fall_angle_rad_)
    {
        fallen_ = true;
        publish_effort(0.0, 0.0);
        return;
    }

    // 刚被扶正：倒地期间轮子空转，位置积分已经飞了，重新标定原点
    if (fallen_)
    {
        fallen_             = false;
        origin_initialized_ = false;
        wheel_velocity_     = 0.0;
        position_target_    = 0.0;
    }
    if (!origin_initialized_)
    {
        position_origin_    = wheel_position_;
        origin_initialized_ = true;
    }

    // 指令超时（遥控没在发）就当停车，别保留上一次的速度指令
    if (command_age_s() > command_timeout_)
    {
        target_velocity_ = 0.0;
        target_yaw_rate_ = 0.0;
    }

    // 目标状态：位置目标由速度指令积分而来，速度目标直接给
    // （让 LQR 追速度，而不是死盯一个位移点 —— 死盯位移的话车会为了「停在原地」
    //   顶着速度指令不动）
    position_target_ += target_velocity_ * dt_;

    State target = State::Zero();
    target(0) = position_target_;
    target(1) = target_velocity_;

    State state;
    state << wheel_position_ - position_origin_, wheel_velocity_, pitch_, pitch_rate_;

    // 公共力矩：俯仰 + 前进。clamp 到 max_effort_，这是「俯仰/前进的预算」
    const double effort =
        std::clamp(control_sign_ * controller_.update(state, target)(0), -max_effort_, max_effort_);

    // 转向：左右轮反向偏置。两轮力矩差产生偏航力矩（正=左转）
    const double differential =
        std::clamp(yaw_sign_ * yaw_kp_ * (target_yaw_rate_ - yaw_rate_), -max_differential_, max_differential_);

    publish_effort(effort - differential, effort + differential);
}

void ChassisNode::publish_effort(double left_effort, double right_effort)
{
    // 单轮上限必须额外留出差分的余量：公共部分最多 max_effort_、转向差分最多
    // max_differential_，所以单轮最大是两者之和。
    // 只夹到 max_effort_ 的话，公共力矩一贴轨，差分正方向那侧就被压平 ——
    // 表现为「直着能站住，一转弯就不听使唤」（lqr 那台实测 88% 的样本如此）
    const double wheel_limit = max_effort_ + max_differential_;

    std_msgs::msg::Float64 left, right;
    left.data  = std::clamp(left_effort,  -wheel_limit, wheel_limit);
    right.data = std::clamp(right_effort, -wheel_limit, wheel_limit);
    command_pub_[kLeft]->publish(left);
    command_pub_[kRight]->publish(right);
}
```

### 4.7 入口 `chassis.cpp`：只改一行

```cpp
auto chassis = std::make_shared<ChassisNode>();   // 参数用 NodeOptions 的默认值

rclcpp::executors::SingleThreadedExecutor executor;
for (const auto& node : chassis->nodes())         // 三个节点（底盘 + 两个电机）
{
    executor.add_node(node);
}
executor.spin();
```

**子节点不加进执行器是这条链上最容易犯的错**：电机节点是裸 `shared_ptr`，不在执行器里
spin 的话它们的订阅和定时器都不会跑，表现是「底盘在发指令，电机一点反应没有」，
而且没有任何报错。`nodes()` 就是干这个的：

```cpp
std::vector<rclcpp::Node::SharedPtr> ChassisNode::nodes()
{
    std::vector<rclcpp::Node::SharedPtr> all;
    all.reserve(kWheelCount + 1);
    all.push_back(shared_from_this());
    for (const auto& motor : motor_)
    {
        all.push_back(motor);
    }
    return all;
}
```

### 4.8 安全三件事（别省）

| 保护 | 判据 | 失效的后果 |
|---|---|---|
| 反馈缺失不发力 | `!has_imu_ \|\| !has_joint_state_` | 拿默认值 0 当状态，等于盲发力 |
| 姿态超限归零 | `\|pitch\| > fall_angle_rad`（0.70 rad = 40°） | 躺着的时候轮子还在全速转，车在地上打转 |
| 指令超时归零 | `command_age_s() > command_timeout_`（0.30 s） | 遥控一断，车按最后一次速度一直冲出去 |
| 倒地后重标定原点 | `fallen_` 复位时 | 扶起来之后位置状态是一个巨大的假值，一放手就冲 |

`command_age_s()` 用 `std::chrono::steady_clock`，**不用仿真时间**：仿真会被暂停/复位，
拿它算「多久没收到指令」会判错。

### 4.9 力矩预算与限幅次序

三个上限，从内到外，**次序不能反**：

| 层 | 值 | 在哪 | 作用 |
|---|---|---|---|
| LQR 公共力矩 | 0.25 N·m | `max_effort`（本文档参数） | 俯仰 + 前进的预算 |
| 偏航差分 | 0.02 N·m | `max_differential` | 转向的预算 |
| 单轮上限 | 0.27 N·m | `max_effort + max_differential` | 上面两个之和 |
| 电机侧 | ≥ 0.27 N·m | `params` 里 motor 的 `max_effort` | 别把它夹回去 |
| URDF 硬限位 | 5.0 N·m | `modules/wheel.xacro` 的 `effort` | gz 的 ApplyJointForce 在这夹断 |

**0.25 N·m 够不够、会不会太大**：

- 附着极限：单轮法向力 ≈ 1.55 × 9.81 / 2 = 7.6 N，`μ1 = 1.0`、`r = 0.04`
  → 打滑前最多 0.30 N·m。**0.25 卡在这个极限之下**，再往上给只是让轮子空转。
- 需求侧：0.25 N·m 对应单轮推力 6.25 N，两个轮子 12.5 N，整车 1.55 kg
  → 等效加速度 **8 m/s²**，远超平衡环需要的量级（压 0.02 rad 静态俯仰偏置只要
  0.045 N·m）。
- 结论：**给到接近附着极限是对的**，不用抠。上限的瓶颈是摩擦不是电机。

---

## 5. 参数文件

### 5.1 `project/params/chassis.yaml` 改成这样

```yaml
# 底盘参数：chassis（平衡控制器）+ 两个电机。改完要 colcon build --packages-select project
#
# 四段：/chassis 是平衡环自己的，/** 是两个电机共用的（通配段，改一处两轮齐变），
# /motor_left 和 /motor_right 只放 max_effort —— 只有它一个键两边都声明、含义还不一样
#
# 数值一律写小数点（1.0 不能写 1）：rclcpp 按 YAML 类型给参数，整数和 double 对不上会抛异常

/chassis:
  ros__parameters:
    # ---- 模型参数：由 URDF 量出来，改模型就要重算，见 docs/轮腿机器人LQR平衡教程.md ----
    wheel_radius: 0.04             # modules/wheel.xacro 的 radius
    cart_mass: 0.6375              # M = 2*(轮子质量 + 轮子绕轴惯量/r^2)
    body_mass: 1.25                # m = 总质量 1.55 - 两轮 0.30
    body_com_height: 0.06825       # l = 车身质心到轮轴
    body_pitch_inertia: 0.002591   # I = 车身绕质心绕 y 轴的俯仰惯量
    cart_damping: 0.02             # b 滚阻/粘滞阻尼，估的量，0~0.05 都行
    gravity: 9.81

    # ---- 控制 ----
    dt: 0.001                      # 控制周期，必须跟物理步长和定时器实际周期一致
    q: [1.0, 1.0, 1000.0, 550.0]   # [位移, 速度, 俯仰, 俯仰角速度] 的对角权重
    r: 1.0                         # 单轮力矩的惩罚权重，越大越柔和
    max_effort: 0.25               # LQR 公共力矩上限。附着极限 0.30，别超过它
    fall_angle_rad: 0.70           # |pitch| 超过它（40°）就认定倒地，力矩归零
    pitch_sign: 1.0                # 俯仰取反用 -1.0
    control_sign: 1.0              # 力矩取反用 -1.0

    # ---- 遥控接口 ----
    cmd_vel_topic: /chassis/cmd_vel
    max_velocity: 0.20             # 前后速度上限 m/s
    max_yaw_rate: 1.0              # 偏航角速度上限 rad/s
    yaw_kp: 0.01                   # 偏航误差 -> 力矩差 N·m·s/rad
    max_differential: 0.02         # 力矩差上限。单轮上限 = 它 + max_effort
    yaw_sign: 1.0                  # 转向方向反了翻成 -1.0
    command_timeout_s: 0.30        # 这么久没收到 cmd_vel 就当停车

    # ---- 反馈话题与模式 ----
    imu_topic: /imu
    joint_states_topic: /joint_states
    control_mode: effort           # 必须是 effort，速度环会和平衡环抢
    # 关节名 / 指令话题 / 力矩话题故意不写在参数里：那是「这台车怎么接线」，
    # 由 chassis 节点统一发给两个电机（见 4.3 的代码），不是手调的量

# 两个电机共用的这一段。这些键只有电机节点声明，别的节点拿到当没看见。
# 别往这里塞别的节点也声明的键（joint_states_topic / max_effort 那种）：
# /** 是当正则匹配的，同一个键被两段命中时靠后的段覆盖靠前的（没有「精确段优先」）
#   joint_states_topic 不写：电机的那个值由底盘覆盖进来（要跟底盘订同一路），写了也不生效
#   speed_kp / speed_ki 不写：只在 speed 模式下有用，而平衡环必须跑 effort
/**:
  ros__parameters:
    max_speed_radps: 60.0          # 力矩模式下只管护栏，不影响出力
    effort_timeout_s: 0.05         # 这么久没收到指令就把力矩清零（平衡环 1 kHz 在发，够用）

# max_effort 是唯一一个两边都声明、含义还不一样的键：底盘的是 LQR 公共力矩上限（0.25），
# 电机的是单轮上限（5.0）。放进 /** 会把底盘那份顶成 5.0，而附着极限才 0.30 ——
# 表现是轮子空转、车瞬间躺下。所以拆成两个显式段
/motor_left:
  ros__parameters:
    max_effort: 5.0                # 单轮力矩上限。必须 >= max_effort + max_differential

/motor_right:
  ros__parameters:
    max_effort: 5.0
```

### 5.2 哪些值必须跟别的文件对齐

改模型/改参数之前先看这张表，**这些数之间没有自动校验，错了不报错只是行为怪**：

| 这个值 | 必须等于 | 在哪 |
|---|---|---|
| `wheel_radius` | `modules/wheel.xacro` 的 `radius` | 模型 |
| `cart_mass` / `body_mass` / `body_com_height` / `body_pitch_inertia` | URDF 量出来的（§1.3 脚本） | 模型 |
| `dt` | gz 物理步长（1 ms，`worlds/empty.sdf`） | 世界 |
| `max_effort` + `max_differential` | ≤ 电机 `max_effort` ≤ URDF `<limit effort>` | 参数 / 模型 |
| `max_effort` + `max_differential` | ≤ `μ1 · (总质量·g/2) · r` ≈ 0.30 | 物理（打滑） |
| `joint_states_topic` | 模型里 `joint_encoder` 的话题（默认 `/joint_states`） | 模型 |
| `imu_topic` | 模型里 `imu_sensor` 的 `topic`（默认 `/imu`） | 模型 |
| 关节名 | `bodys/wheel_leg_robot.xacro` 里 `joint_name` | 模型 |
| 电机的 `force_topic` | launch 里桥接的重映射目标 | launch |

---

## 6. launch 与桥

### 6.1 桥：不用动

`project/launch/sim.launch.py` 里的桥接已经配好了这四路，**写平衡环时一行都不用改**：

| 方向 | ROS 侧 | gz 侧 |
|---|---|---|
| ROS → gz | `/motor/left/cmd_force` | `/model/wheel_leg_robot/joint/left_wheel_joint/cmd_force` |
| ROS → gz | `/motor/right/cmd_force` | `/model/wheel_leg_robot/joint/right_wheel_joint/cmd_force` |
| gz → ROS | `/joint_states` | 同名 |
| gz → ROS | `/imu` | 同名 |
| gz → ROS | `/clock` | 同名 |

两个要留意的点：

- **gz 侧的话题名里焊死了模型名**（`ApplyJointForce` 插件自己拼的），所以 ROS 侧只能
  用重映射改成短名。模型改名（`MODEL_NAME`）时这两个字符串会一起变，因为 launch 里是
  用 f-string 拼的。
- **`/clock` 必须有**：电机节点的速度环要用 `now()` 算 dt，没有 `/clock` 时 `use_sim_time`
  下 `now()` 恒为 0。本文的平衡环用 `timer` 的固定 `dt_`，不依赖它 —— 但电机那边要。

### 6.2 节点那两段

**chassis 那段已经解开了**（连同 §4.1 的重写），现在长这样：

```python
        # 底盘：一个进程里三个节点（chassis 平衡环 + 两个电机）
        Node(package='project',
             executable='chassis',
             parameters=[chassis_params, {'use_sim_time': True}],
             output='screen'),
```

其中 `chassis_params` 指到 install 空间里那份：

```python
chassis_params = os.path.join(get_package_share_directory('project'), 'params', 'chassis.yaml')
```

imu 节点那段**仍然停着**（平衡环直接订 `/imu`，见 §3.3）。什么时候要解：想让
`/imu/euler_deg` 之类的话题能 `ros2 topic echo` 出来看姿态的时候。

### 6.3 全链路

```
gz 物理(1ms)
  ├─ JointStatePublisher(/joint_states) ──桥──> /joint_states ─┐
  └─ Imu(/imu) ──────────────────────────桥──> /imu ──────────┤
                                                              ↓
                                          chassis（平衡环 1 kHz）
                                            LQR: x -> τ          │
                                                              ↓
                                       /motor/left|right/command (Float64, N·m)
                                                              ↓
                                       motor_left|right（MotorNode，effort 模式）
                                                              ↓
                                       /motor/left|right/cmd_force
                                                              ↓
                                                          桥 ──> gz 关节力矩
```

键盘遥控在另一头：`ros2 run project keyboard` → `/chassis/cmd_vel` → chassis。

---

## 7. 上手顺序与验收

### 7.1 分步

```bash
# 0) 模型改了就必须重新 build：launch 读的是 install 空间里那份拷贝
cd /home/qingyudd01/projects/vscode/gazebo/try
colcon build --packages-select project && source install/setup.bash

# 1) 只起仿真和桥（节点先别起），确认模型和反馈
ros2 launch project sim.launch.py
#   另一个终端：
source install/setup.bash
ros2 topic hz /imu              # 期望 ~1000 Hz
ros2 topic hz /joint_states     # 期望 ~1000 Hz
ros2 topic echo /imu --once     # orientation.w 接近 1，angular_velocity 接近 0
ros2 topic echo /joint_states --once | grep -A2 name   # 能看到 left_wheel_joint / right_wheel_joint

# 2) 起平衡环（chassis 那段解开之后）
ros2 launch project sim.launch.py
#   日志里应该有一行： LQR K = [-0.045481, -0.193570, -2.254953, -1.083929]
#   拿它跟 §2.4 对一下

# 3) 看它站没站住（gz 界面里），另开一个终端看输出
ros2 topic hz /motor/left/command     # 实际控制周期，应该 ~1000 Hz
ros2 topic echo /motor/left/command   # 静止站立时是小力矩（±0.05 N·m 量级）

# 4) 推一把：键盘遥控
ros2 run project keyboard             # 要在有终端的窗口里前台跑
```

### 7.2 自检清单

- [ ] xacro 能展开（`xacro <body>.xacro > /dev/null` 不报错），两个 `*_wheel_joint` 是
      `continuous`、`<limit effort>` ≥ 单轮上限
- [ ] `/joint_states` 里有 `left_wheel_joint` / `right_wheel_joint`
- [ ] `/imu` 的 `orientation.w ≈ 1`、`angular_velocity` 静止时 ≈ 0
- [ ] 车 spawn 在 z = 0.04，落地时 `pitch ≈ 0`（不是 0 就是 spawn 高度不对）
- [ ] `LQR K` 那行 4 个数，和 §2.4 对得上，**绝对值最大的是俯仰项**
- [ ] 起平衡环之后车能站住（不发力时它是自己倒的，发力后能回正）
- [ ] 用手推一下能回正，来回摆两下就停
- [ ] 给一个小的 `cmd_vel`，车能慢慢往前走
- [ ] Ctrl-C 掉平衡环之后，电机在 50 ms 内停止出力（`effort_timeout_s` 生效）

### 7.3 现象 → 上哪查

| 现象 | 大概率原因 |
|---|---|
| 一发力就往一边加速倒下，越倒越猛 | `pitch_sign` / `control_sign` 符号错（§2.5） |
| 刚起步就疯狂往一个方向冲 | 忘了 `position_origin_` 重标定（§3.1） |
| 站着高频抖 | `r` 太小 / `dt` 跟实际周期不一致 / 轮子摩擦不足 |
| 直着能站住，一转弯就不听使唤 | 单轮上限夹成了 `max_effort_`，没给差分留余量（§4.6） |
| 一直朝一个方向慢漂 | 正常（里程计漂），加大 `q[0]`、`q[1]` 能压住 |
| K 打印出来只有 3 个数 | `q` 数组长度不对 |
| 电机一点反应没有 | 子节点没加进 executor（§4.7） |
| 力矩发出去车纹丝不动 | 桥没起 / 关节名不对 / `control_mode` 不是 effort |
| 站起来之后腿的姿态变了 | 腿被解开了（`fixed` → `revolute`），这一组 K 不再适用（§1.4） |

### 7.4 Q/R 怎么调

顺序很重要，**先让它站住，再谈站得好不好**：

1. `q[2]`（俯仰）先给大（1000），`q[3]`（俯仰角速度）给它的 0.5~1 倍 —— 这一对决定
   回正快不快、过不过冲。
2. `q[0]`、`q[1]`（位移、速度）给小（1.0）。给大了会为了「守住位置」跟速度指令顶牛，
   表现为车在原地犹豫。
3. `r` 从 1.0 起步。抖就往上加（10、100），响应太肉就往下减。
4. 每次改完看 `LQR K` 那行：改 Q/R 只该让 K 平滑地变，**K 跳变说明参数写错了**。

改完 Q/R 只需要重启 chassis 节点，不用重新 build（YAML 在 install 空间，但 launch 是
现读的 —— 前提是你改的是 `install` 里那份；改源码那份要 build）。

---

## 附录：物理量速查（本文用到的所有数）

| 量 | 值 | 出处 |
|---|---|---|
| 整车质量 | 1.5500 kg | URDF |
| 轮子半径 / 宽 / 质量 | 0.04 m / 0.02 m / 0.15 kg | `modules/wheel.xacro` |
| 轮子绕轴惯量 | 0.00027 kg·m² | 同上（`izz`，不是 `iyy`） |
| 轮距（左右轮心距） | 0.12 m | `bodys/wheel_leg_robot.xacro` |
| 腿长 / 标准姿态角 | 0.056 m / 40° | `modules/leg.xacro` |
| 胯轴高度（轮轴上方） | 0.112 m | `2·leg_drop` |
| IMU 高度（轮轴上方） | 0.092 m | `hip_height + 0.02` |
| 底盘尺寸 / 质量 | 110×90×30 mm / 1.11 kg | `modules/chassis.xacro` |
| 单轮法向力（静止） | 7.60 N | 1.55·9.81/2 |
| 单轮附着极限力矩 | 0.30 N·m | `μ1·N·r` |
| 物理步长 | 1 ms | `worlds/empty.sdf` |
| 控制/传感/电机节拍 | 1 kHz | 全链一致 |
