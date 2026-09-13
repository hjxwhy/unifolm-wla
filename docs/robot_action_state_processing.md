# 机器人动作、状态与统计量处理规范
## 摘要

本规范将不同机器人本体的数据映射到统一的动作空间和状态空间：

- 每个未来动作表示为 54 维向量；
- 每个当前状态表示为 60 维向量；
- 使用布尔掩码标识实际存在的模块；
- 末端执行器和底盘位姿动作表示为相对当前状态的 SE(3) 变换；
- 其他动作保留数据源中的控制语义；
- 相对位姿动作默认使用全局 Z-score；
- 普通动作和状态默认使用 1%/99% 分位数归一化；
- 同一本体机器人的不同任务统计量采用等任务权重合并，双臂末端还可以进一步合并左右手统计量。

本文中的“必须”“应”“不得”表示复现本数据处理流程时需要遵守的规范要求。

---

## 1. 符号、数据类型与基本约定

### 1.1 时间与维度符号

| 符号 | 含义 |
|---|---|
| $t$ | 当前观测时刻，即动作块的参考时刻 |
| $k$ | 动作块内的预测步索引，$k=0,\ldots,H-1$ |
| $H$ | 动作块长度 |
| $f_s$ | 数据源帧率 |
| $f_t$ | 目标帧率 |
| $D_a=54$ | 统一动作维数 |
| $D_s=60$ | 统一状态维数 |
| $\mathbf a_{t,k}$ | 当前时刻 $t$ 对应的第 $k$ 个未来动作 |
| $\mathbf s_t$ | 当前时刻状态 |
| $\mathbf m^a$ | 54 维动作有效性掩码 |
| $\mathbf m^s$ | 60 维状态有效性掩码 |

默认配置为：

```yaml
target_fps: 30
chunk_size: 30
norm_type: minmax_q
rel_norm_type: zscore
state_norm_type: minmax_q
gripper_norm_type: minmax_q
```

因此，默认动作块包含 30 个动作点，覆盖区间约为 $[0,29/30]$ 秒。

### 1.2 数值类型

统一动作、统一状态、offset 和 scale 均应保存为 `float32`。统计量计算阶段可以使用 `float64` 以降低合并误差，最终再转换为 `float32`。

掩码使用布尔类型：

```math
\mathbf m^a\in\{0,1\}^{54},
\qquad
\mathbf m^s\in\{0,1\}^{60}.
```

### 1.3 坐标系和单位约定

#### 1.3.1 机器人坐标系定义

机器人 base 坐标系采用右手坐标系，记为 $B$：

| 坐标轴 | 正方向 |
|---|---|
| $x$ | forward，机器人正前方 |
| $y$ | left，机器人左侧 |
| $z$ | up，机器人正上方 |

坐标轴关系满足：

```math
\mathbf e_x\times\mathbf e_y=\mathbf e_z.
```

左、右末端执行器的绝对位姿分别记为 $T^B_{E_L}$ 和 $T^B_{E_R}$，均在机器人 base 坐标系 $B$ 中表达。因此双臂末端位置和旋转使用同一套坐标轴定义：$x$ 向前、$y$ 向左、$z$ 向上。右臂不得使用镜像坐标轴，左右臂同一维度的正负方向必须具有相同物理含义。

末端执行器自身的瞬时朝向由位姿中的旋转矩阵 $R^B_E$ 描述；上述约定定义的是末端绝对位姿的参考坐标系，而不是要求末端局部坐标轴在运动过程中始终与 base 坐标轴平行。

#### 1.3.2 单位与数值约定

数据处理程序本身不执行单位换算。所有待合并数据必须预先统一：

- 平移单位：米；
- 旋转角单位：弧度；
- 线速度单位：米每秒；
- 角速度单位：弧度每秒；
- 四元数顺序：`xyzw`；
- 欧拉角顺序：`xyz`；
- 左右手关节顺序和夹爪开合方向必须在数据集级别保持一致。

如果输入数据不满足这些约定，应在统计和映射之前完成转换。

---

## 2. 完整处理流程

对每个当前时刻 $t$，按以下顺序生成模型输入和监督信号：

1. 读取当前状态；
2. 将当前状态转换为统一的 60 维表示；
3. 按模块归一化状态；
4. 读取未来动作时间窗；
5. 计算末端执行器和底盘位姿的相对动作；
6. 对各动作模块分别归一化；
7. 可选地对夹爪动作进行二值化；
8. 将动作序列从源帧率重采样到目标帧率；
9. 将各动作模块填入统一的 54 维表示；
10. 生成动作和状态有效性掩码；
11. 输出反归一化所需的 54 维动作 offset 和 scale。

整体关系可写为：

```math
\text{原始状态}
\longrightarrow
\text{状态格式转换}
\longrightarrow
\text{状态归一化}
\longrightarrow
\mathbf s_t,
```

```math
(\text{当前状态},\text{未来动作})
\longrightarrow
\text{相对动作}
\longrightarrow
\text{动作归一化}
\longrightarrow
\text{重采样}
\longrightarrow
\mathbf A_t.
```

其中

```math
\mathbf A_t=
\begin{bmatrix}
\mathbf a_{t,0}^{\mathsf T}\\
\mathbf a_{t,1}^{\mathsf T}\\
\vdots\\
\mathbf a_{t,H-1}^{\mathsf T}
\end{bmatrix}
\in\mathbb R^{H\times54}.
```

---

## 3. 统一动作表示

### 3.1 54 维动作布局

| 索引 | 维数 | 模块 | 语义 | 规范表示 |
|---|---:|---|---|---|
| `[0:6]` | 6 | 左末端执行器 | 相对当前左末端状态的未来位姿 | 相对 xyz + rotation vector |
| `[6:7]` | 1 | 左夹爪 | 未来夹爪命令 | 原始标量，归一化后可二值化 |
| `[7:13]` | 6 | 左灵巧手 | 未来手部控制命令 | 6 维原始动作 |
| `[13:19]` | 6 | 右末端执行器 | 相对当前右末端状态的未来位姿 | 相对 xyz + rotation vector |
| `[19:20]` | 1 | 右夹爪 | 未来夹爪命令 | 原始标量，归一化后可二值化 |
| `[20:26]` | 6 | 右灵巧手 | 未来手部控制命令 | 6 维原始动作 |
| `[26:29]` | 3 | 腰部 | 未来腰部动作 | 前 3 个动作分量 |
| `[29:32]` | 3 | 躯干 | 未来躯干动作 | 前 3 个动作分量 |
| `[32:34]` | 2 | 底盘平移 | 未来底盘线速度命令 | $v_x,v_y$ |
| `[34:35]` | 1 | 底盘转动 | 未来底盘偏航角速度命令 | $\omega_z$ |
| `[35:41]` | 6 | 底盘位姿 | 相对当前底盘状态的未来位姿 | 相对 xyz + rotation vector |
| `[41:42]` | 1 | 高度 | 未来高度命令 | 标量 |
| `[42:48]` | 6 | 左腿 | 未来左腿关节动作 | 前 6 个动作分量 |
| `[48:54]` | 6 | 右腿 | 未来右腿关节动作 | 前 6 个动作分量 |

### 3.2 末端动作分量

左、右末端动作均采用以下 6 维排列：

```math
\mathbf a^{ee}_{t,k}
=
[\Delta x,\Delta y,\Delta z,\phi_x,\phi_y,\phi_z]^{\mathsf T}.
```

前三维是当前末端坐标系下的相对平移，后三维是相对旋转的旋转向量。

旋转向量定义为：

```math
\boldsymbol\phi=\theta\mathbf u,
```

其中 $\mathbf u$ 是单位旋转轴，$\theta$ 是旋转角。因此：

```math
\|\boldsymbol\phi\|_2=\theta.
```

### 3.3 非位姿动作

以下动作不执行 SE(3) 相对化，而是直接读取未来动作值：

- 夹爪；
- 左右灵巧手；
- 腰部；
- 躯干；
- 底盘速度；
- 高度；
- 左右腿。

这些字段可以表示绝对目标、速度命令或已经计算好的增量，具体语义由数据源定义。为了实现跨数据集训练，所有被映射到同一槽位的数据必须具有相同控制语义。

### 3.4 动作填充规则

初始化：

```math
\mathbf a_{t,k}=\mathbf 0\in\mathbb R^{54},
\qquad
\mathbf m^a=\mathbf 0\in\{0,1\}^{54}.
```

对当前机器人存在的模块：

1. 将模块数值写入固定槽位；
2. 将该槽位对应的掩码设为 1。

不存在的模块保持数值 0、掩码 0。

如果输入动作模块维数大于槽位维数，只保留前若干维；如果维数小于槽位维数，则视为数据模式错误，不对动作块自动补齐。因而每个已启用动作模块必须至少提供对应槽位所需的维数。

---

## 4. 统一状态表示

### 4.1 60 维状态布局

| 索引 | 维数 | 模块 | 语义 | 规范表示 |
|---|---:|---|---|---|
| `[0:9]` | 9 | 左末端执行器 | 当前绝对位姿 | xyz + rotation-6D |
| `[9:10]` | 1 | 左夹爪 | 当前夹爪状态 | 标量 |
| `[10:16]` | 6 | 左灵巧手 | 当前手部状态 | 6 维 |
| `[16:25]` | 9 | 右末端执行器 | 当前绝对位姿 | xyz + rotation-6D |
| `[25:26]` | 1 | 右夹爪 | 当前夹爪状态 | 标量 |
| `[26:32]` | 6 | 右灵巧手 | 当前手部状态 | 6 维 |
| `[32:35]` | 3 | 腰部 | 当前腰部关节状态 | 前 3 维 |
| `[35:38]` | 3 | 躯干 | 当前躯干关节状态 | 前 3 维 |
| `[38:40]` | 2 | 底盘平移速度 | 当前 $v_x,v_y$ | 2 维 |
| `[40:41]` | 1 | 底盘角速度 | 当前 $\omega_z$ | 预留槽位 |
| `[41:47]` | 6 | 底盘惯性状态 | 机体坐标系下的重力方向与角速度 | $[g_x^B,g_y^B,g_z^B,\hat\omega_x^B,\hat\omega_y^B,\hat\omega_z^B]$ |
| `[47:48]` | 1 | 高度 | 当前高度 | 预留槽位 |
| `[48:54]` | 6 | 左腿 | 当前左腿关节状态 | 前 6 维 |
| `[54:60]` | 6 | 右腿 | 当前右腿关节状态 | 前 6 维 |

状态始终表示当前时刻的绝对观测，不对当前状态自身做相对化。

### 4.2 末端状态的 rotation-6D

设旋转矩阵为

```math
R=
\begin{bmatrix}
R_{00}&R_{01}&R_{02}\\
R_{10}&R_{11}&R_{12}\\
R_{20}&R_{21}&R_{22}
\end{bmatrix}.
```

本规范使用旋转矩阵前两列构造 rotation-6D：

```math
\rho_6(R)=
[R_{00},R_{10},R_{20},R_{01},R_{11},R_{21}]^{\mathsf T}.
```

因此末端绝对状态为：

```math
\mathbf s^{ee}_t=
[x,y,z,\rho_6(R_t)^{\mathsf T}]^{\mathsf T}
\in\mathbb R^9.
```

如果需要从一般的两个三维向量 $\mathbf a_1,\mathbf a_2$ 恢复旋转矩阵，使用 Gram–Schmidt 正交化：

```math
\mathbf b_1=
\frac{\mathbf a_1}{\|\mathbf a_1\|_2},
```

```math
\widetilde{\mathbf b}_2
=
\mathbf a_2-(\mathbf b_1^{\mathsf T}\mathbf a_2)\mathbf b_1,
```

```math
\mathbf b_2=
\frac{\widetilde{\mathbf b}_2}
{\|\widetilde{\mathbf b}_2\|_2},
\qquad
\mathbf b_3=\mathbf b_1\times\mathbf b_2,
```

```math
R=[\mathbf b_1,\mathbf b_2,\mathbf b_3].
```

所有数据生成、训练和部署模块必须使用相同的“前两列”约定。

### 4.3 底盘重力方向与角速度状态

状态槽位 `[41:47]` 不表示底盘绝对位姿，而表示机体坐标系 $B$ 下的重力方向和角速度：

```math
\boxed{
\mathbf s_t^{base}=
[
g_x^B,\,
g_y^B,\,
g_z^B,\,
\hat\omega_x^B,\,
\hat\omega_y^B,\,
\hat\omega_z^B
]^{\mathsf T}
}
```

其中：

- $\mathbf g^B=[g_x^B,g_y^B,g_z^B]^{\mathsf T}$ 是机体坐标系中的单位重力方向；
- $\boldsymbol\omega^B=[\omega_x^B,\omega_y^B,\omega_z^B]^{\mathsf T}$ 是机体坐标系中的原始三轴角速度；
- $\widehat{\boldsymbol\omega}^B$ 是归一化后的三轴角速度。

#### 4.3.1 重力方向计算

设 $R_{WB}$ 表示把机体坐标系向量转换到世界坐标系的旋转矩阵，世界坐标系中的单位重力方向定义为：

```math
\mathbf g^W=[0,0,-1]^{\mathsf T}.
```

则机体坐标系中的重力方向为：

```math
\mathbf g^B=R_{WB}^{\mathsf T}\mathbf g^W.
```

为消除输入四元数或旋转矩阵的数值误差，写入状态前再次单位化：

```math
\mathbf g^B
\leftarrow
\frac{\mathbf g^B}
{\max(\|\mathbf g^B\|_2,\epsilon)},
```

其中建议取 $\epsilon=10^{-8}$。因此理论上应满足：

```math
\|\mathbf g^B\|_2=1.
```

重力方向只编码机体相对于重力方向的俯仰和横滚信息，不包含世界坐标系中的绝对位置，也不提供绕重力轴的绝对偏航角。

#### 4.3.2 角速度归一化

原始机体角速度为：

```math
\boldsymbol\omega^B=
[\omega_x^B,\omega_y^B,\omega_z^B]^{\mathsf T}.
```

按照状态角速度统计量进行逐维归一化：

```math
\widehat{\boldsymbol\omega}^B
=
\frac{
\boldsymbol\omega^B-\mathbf o_{\omega}
}{
\mathbf c_{\omega}
}.
```

默认使用 `state_norm_type=minmax_q`，因此：

```math
\mathbf o_{\omega}
=
\frac{
Q_{0.01}(\boldsymbol\omega^B)+
Q_{0.99}(\boldsymbol\omega^B)
}{2},
```

```math
\mathbf c_{\omega}
=
\frac{
Q_{0.99}(\boldsymbol\omega^B)-
Q_{0.01}(\boldsymbol\omega^B)
}{2}.
```

若数据入口已经提供 $\widehat{\boldsymbol\omega}^B$，则直接写入 `[44:47]`，不得重复归一化。

#### 4.3.3 槽位映射

```math
[g_x^B,g_y^B,g_z^B]
\longrightarrow[41:44],
```

```math
[\hat\omega_x^B,\hat\omega_y^B,\hat\omega_z^B]
\longrightarrow[44:47].
```

该状态块不包含底盘 xyz 位置，也不包含 rotation vector。动作空间 `[35:41]` 的底盘动作定义保持不变，仍然表示相对底盘位姿的 xyz + rotation vector。

### 4.4 状态填充与掩码

状态向量和状态掩码初始化为：

```math
\mathbf s_t=\mathbf 0\in\mathbb R^{60},
\qquad
\mathbf m^s=\mathbf 0\in\{0,1\}^{60}.
```

存在的状态模块写入固定槽位，并将对应掩码置为 1；不存在的模块保持为 0。若状态模块短于目标槽位，则在尾部补 0，并仍将整个槽位标为有效；若长于目标槽位，则只保留槽位允许的前若干维。因此，状态字段维数也应在数据接入阶段严格校验。

---

## 5. 位姿格式转换

所有相对位姿计算必须先将输入转换为 SE(3) 齐次变换：

```math
T=
\begin{bmatrix}
R&\mathbf p\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix}.
```

其中 $\mathbf p=[x,y,z]^{\mathsf T}$。

### 5.1 xyz + RPY

输入排列为：

```math
[x,y,z,r,p,y].
```

这里最后一个 $y$ 表示 yaw。为避免符号混淆，下文写为 $r,p,\psi$。

定义：

```math
R_x(r)=
\begin{bmatrix}
1&0&0\\
0&\cos r&-\sin r\\
0&\sin r&\cos r
\end{bmatrix},
```

```math
R_y(p)=
\begin{bmatrix}
\cos p&0&\sin p\\
0&1&0\\
-\sin p&0&\cos p
\end{bmatrix},
```

```math
R_z(\psi)=
\begin{bmatrix}
\cos\psi&-\sin\psi&0\\
\sin\psi&\cos\psi&0\\
0&0&1
\end{bmatrix}.
```

采用固定轴 `xyz` 欧拉角约定：

```math
R=R_z(\psi)R_y(p)R_x(r).
```

所有角度均为弧度。

### 5.2 xyz + quaternion

输入排列为：

```math
[x,y,z,q_x,q_y,q_z,q_w].
```

首先归一化四元数：

```math
\bar{\mathbf q}=
\frac{\mathbf q}{\|\mathbf q\|_2}.
```

令归一化后的四元数仍记为 $(q_x,q_y,q_z,q_w)$，旋转矩阵为：

```math
R=
\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_zq_w) & 2(q_xq_z+q_yq_w)\\
2(q_xq_y+q_zq_w) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_xq_w)\\
2(q_xq_z-q_yq_w) & 2(q_yq_z+q_xq_w) & 1-2(q_x^2+q_y^2)
\end{bmatrix}.
```

### 5.3 xyz + rotation vector

输入排列为：

```math
[x,y,z,\phi_x,\phi_y,\phi_z].
```

令

```math
\boldsymbol\phi=[\phi_x,\phi_y,\phi_z]^{\mathsf T},
\qquad
\theta=\|\boldsymbol\phi\|_2.
```

当 $\theta>0$ 时，令 $\mathbf u=\boldsymbol\phi/\theta$，通过 Rodrigues 公式计算：

```math
R=
I+\sin\theta[\mathbf u]_{\times}
+(1-\cos\theta)[\mathbf u]_{\times}^2.
```

当 $\theta$ 接近 0 时，应使用稳定的小角度展开或成熟的 SO(3) 实现。

---

## 6. Relative action 的定义与计算

### 6.1 末端与底盘相对位姿

设当前状态位姿为：

```math
T_t=
\begin{bmatrix}
R_t&\mathbf p_t\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix},
```

第 $k$ 个未来动作目标为：

```math
T_{t+k}=
\begin{bmatrix}
R_{t+k}&\mathbf p_{t+k}\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix}.
```

当前位姿的逆为：

```math
T_t^{-1}=
\begin{bmatrix}
R_t^{\mathsf T}&-R_t^{\mathsf T}\mathbf p_t\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix}.
```

相对动作定义为：

```math
T^{rel}_{t,k}=T_t^{-1}T_{t+k}.
```

展开可得：

```math
R^{rel}_{t,k}=R_t^{\mathsf T}R_{t+k},
```

```math
\mathbf p^{rel}_{t,k}
=R_t^{\mathsf T}(\mathbf p_{t+k}-\mathbf p_t).
```

因此，相对平移位于当前末端或当前底盘的局部坐标系中，而不是世界坐标系中的直接位置差。

最终将相对旋转矩阵转换为旋转向量：

```math
\boldsymbol\phi^{rel}_{t,k}
=\mathrm{Log}(R^{rel}_{t,k})^{\vee}.
```

输出相对动作：

```math
\mathbf a^{rel}_{t,k}
=
\begin{bmatrix}
\mathbf p^{rel}_{t,k}\\
\boldsymbol\phi^{rel}_{t,k}
\end{bmatrix}
\in\mathbb R^6.
```

### 6.2 相对动作伪代码

```text
function relative_pose(current_pose, future_poses, pose_format):
    T_current = pose_to_SE3(current_pose, pose_format)
    T_current_inverse = inverse_SE3(T_current)

    result = []
    for future_pose in future_poses:
        T_future = pose_to_SE3(future_pose, pose_format)
        T_relative = T_current_inverse @ T_future

        relative_xyz = T_relative[0:3, 3]
        relative_rotvec = SO3_log(T_relative[0:3, 0:3])
        result.append(concat(relative_xyz, relative_rotvec))

    return result
```

---

## 7. Relative action 的全局统计

项目只使用全局统计量，不按动作块中的时间位置分别建立统计量。

### 7.1 全局统计方法

对数据集中的每个有效当前状态，按照第 6 节的方法计算整个未来动作块：

```math
\mathbf A^{(n)}
=
\begin{bmatrix}
\mathbf a^{rel}_{n,0}\\
\mathbf a^{rel}_{n,1}\\
\vdots\\
\mathbf a^{rel}_{n,H-1}
\end{bmatrix}
\in\mathbb R^{H\times d},
```

其中 $n=1,\ldots,N$ 表示不同样本，$H$ 是动作块长度，末端或底盘相对位姿的维数为 $d=6$。

将所有样本和所有未来时间步合并为同一个二维数组：

```math
X_{rel}
=
\mathrm{reshape}
\left(
\{\mathbf A^{(n)}\}_{n=1}^{N},
(NH,d)
\right).
```

也就是说，每个动作块中的每一个相对动作点都被视为一个独立统计样本，不区分它位于动作块的第几步。

对第 $j$ 个动作维度，全局均值为：

```math
\mu^{global}_j
=
\frac{1}{NH}
\sum_{n=1}^{N}
\sum_{k=0}^{H-1}
A^{(n)}_{k,j}.
```

全局总体标准差为：

```math
\sigma^{global}_j
=
\sqrt{
\frac{1}{NH}
\sum_{n=1}^{N}
\sum_{k=0}^{H-1}
\left(
A^{(n)}_{k,j}-\mu^{global}_j
\right)^2
}.
```

全局最小值和最大值为：

```math
x^{global}_{min,j}
=
\min_{n,k}A^{(n)}_{k,j},
```

```math
x^{global}_{max,j}
=
\max_{n,k}A^{(n)}_{k,j}.
```

全局 1% 和 99% 分位数为：

```math
Q^{global}_{0.01,j}
=
Q_{0.01}
\left(
\{A^{(n)}_{k,j}\}_{n,k}
\right),
```

```math
Q^{global}_{0.99,j}
=
Q_{0.99}
\left(
\{A^{(n)}_{k,j}\}_{n,k}
\right).
```

每个全局统计量的形状均为：

```math
(d,).
```

相对位姿默认采用 Z-score，因此实际归一化使用：

```math
\widehat{\mathbf a}^{rel}_{n,k}
=
\frac{
\mathbf a^{rel}_{n,k}-
\boldsymbol\mu^{global}
}{
\boldsymbol\sigma^{global}
}.
```

同一组 $\boldsymbol\mu^{global}$ 和 $\boldsymbol\sigma^{global}$ 应用于动作块的全部时间步。

### 7.2 统计数据格式

每个相对动作模块只需要保存以下全局统计量：

```json
{
  "relative_action_key": {
    "global_max": [0.0],
    "global_min": [0.0],
    "global_q01": [0.0],
    "global_q99": [0.0],
    "global_mean": [0.0],
    "global_std": [0.0]
  }
}
```

对于 6 维相对位姿，上述每个数组的长度均为 6，排列顺序为：

```math
[\Delta x,\Delta y,\Delta z,
\phi_x,\phi_y,\phi_z].
```

### 7.3 统计生成伪代码

```text
function collect_global_relative_statistics(samples):
    all_relative_steps = []

    for sample in samples:
        relative_chunk = relative_pose(
            sample.current_pose,
            sample.future_pose_chunk,
            sample.pose_format,
        )

        for relative_action in relative_chunk:
            all_relative_steps.append(relative_action)

    X = stack(all_relative_steps)  # shape: (number_of_all_steps, action_dim)

    return {
        "global_max": max(X, axis=0),
        "global_min": min(X, axis=0),
        "global_q01": quantile(X, 0.01, axis=0),
        "global_q99": quantile(X, 0.99, axis=0),
        "global_mean": mean(X, axis=0),
        "global_std": population_std(X, axis=0)
    }
```

---

## 8. 普通状态与动作统计

对于不需要在线相对化的字段，将全部数据记录中的低维向量堆叠为：

```math
X=
\begin{bmatrix}
\mathbf x_1^{\mathsf T}\\
\mathbf x_2^{\mathsf T}\\
\vdots\\
\mathbf x_M^{\mathsf T}
\end{bmatrix}
\in\mathbb R^{M\times d}.
```

逐维计算：

```math
\boldsymbol\mu
=
\frac{1}{M}\sum_{i=1}^{M}\mathbf x_i,
```

```math
\boldsymbol\sigma
=
\sqrt{
\frac{1}{M}
\sum_{i=1}^{M}
(\mathbf x_i-\boldsymbol\mu)^2
},
```

以及：

```math
\mathbf x_{min},\quad
\mathbf x_{max},\quad
Q_{0.01}(X),\quad
Q_{0.99}(X).
```

普通统计文件中的每个字段至少应包含：

```json
{
  "feature_key": {
    "mean": [0.0],
    "std": [1.0],
    "min": [-1.0],
    "max": [1.0],
    "q01": [-0.9],
    "q99": [0.9]
  }
}
```

可选保存 `count`，用于未来实现样本数加权合并。

---

## 9. 归一化定义

### 9.1 统一仿射形式

所有连续量统一使用：

```math
\widehat{\mathbf x}
=
\frac{\mathbf x-\mathbf o}{\mathbf c},
```

其中除法为逐元素运算。反归一化为：

```math
\mathbf x
=
\widehat{\mathbf x}\odot\mathbf c+
\mathbf o.
```

归一化后不执行裁剪，因此超出统计范围的数据可以小于 $-1$ 或大于 $1$。

任意 scale 分量满足以下保护规则：

```math
c_j=
\begin{cases}
1,&c_j<10^{-6},\\
c_j,&\text{其他情况}.
\end{cases}
```

如果某个模块没有统计量，使用恒等变换：

```math
\mathbf o=\mathbf 0,
\qquad
\mathbf c=\mathbf 1.
```

### 9.2 分位数归一化 `minmax_q`

统计量选择优先级为：

1. `global_q01/global_q99`；
2. `q01/q99`；
3. `min/max`；
4. 恒等变换。

令下界和上界为 $\mathbf l,\mathbf h$，则：

```math
\mathbf o=
\frac{\mathbf l+\mathbf h}{2},
```

```math
\mathbf c=
\frac{\mathbf h-\mathbf l}{2}.
```

因此：

```math
\mathbf l\mapsto-1,
\qquad
\mathbf h\mapsto1.
```

### 9.3 Z-score 归一化 `zscore`

统计量选择优先级为：

1. `global_mean/global_std`；
2. `mean/std`；
3. 恒等变换。

参数为：

```math
\mathbf o=\boldsymbol\mu,
\qquad
\mathbf c=\boldsymbol\sigma.
```

### 9.4 极值归一化 `minmax`

统计量选择优先级为：

1. `global_min/global_max`；
2. `min/max`；
3. 恒等变换。

参数为：

```math
\mathbf o=
\frac{\mathbf x_{min}+\mathbf x_{max}}{2},
```

```math
\mathbf c=
\frac{\mathbf x_{max}-\mathbf x_{min}}{2}.
```

---

## 10. 每个动作模块使用的归一化量

默认归一化配置为：

```yaml
norm_type: minmax_q
rel_norm_type: zscore
gripper_norm_type: minmax_q
```

| 动作模块 | 统一槽位 | 统计来源 | 归一化类型 | 实际优先使用的统计量 |
|---|---|---|---|---|
| 单臂末端相对位姿 | `[0:6]` | 相对动作统计 | `rel_norm_type` | `global_mean/global_std` |
| 左末端相对位姿 | `[0:6]` | 相对动作统计 | `rel_norm_type` | `global_mean/global_std` |
| 右末端相对位姿 | `[13:19]` | 相对动作统计 | `rel_norm_type` | `global_mean/global_std` |
| 底盘相对位姿 | `[35:41]` | 相对动作统计 | `rel_norm_type` | `global_mean/global_std` |
| 单臂/左/右夹爪 | `[6:7]`, `[19:20]` | 普通动作统计 | `gripper_norm_type` | `q01/q99` |
| 左右灵巧手 | `[7:13]`, `[20:26]` | 普通动作统计 | `gripper_norm_type` | `q01/q99` |
| 腰部 | `[26:29]` | 普通动作统计 | `norm_type` | `q01/q99` |
| 躯干 | `[29:32]` | 普通动作统计 | `norm_type` | `q01/q99` |
| 底盘 $v_x,v_y$ | `[32:34]` | 完整 `base_command` 的普通统计 | `norm_type` | 对应源维度的 `q01/q99` |
| 底盘 $\omega_z$ | `[34:35]` | 完整 `base_command` 的普通统计 | `norm_type` | `vyaw` 或 `vw` 对应维度 |
| 高度命令 | `[41:42]` | 完整 `base_command` 的普通统计 | `norm_type` | `height` 对应维度 |
| 左右腿 | `[42:54]` | 普通动作统计 | `norm_type` | `q01/q99` |

注意：

1. 灵巧手使用夹爪归一化类型，而不是普通动作归一化类型；
2. 末端和底盘相对位姿必须使用相对动作统计，不能使用绝对位姿统计；
3. `base_command` 先按完整源向量生成统计量，再按维度映射表抽取 offset 和 scale；
4. 统一动作空间中不存在的模块使用 offset=0、scale=1。

### 10.1 `base_command` 统计量映射

假设源底盘命令为：

```math
\mathbf b=[b_0,b_1,\ldots,b_{d-1}]^{\mathsf T},
```

并给定索引映射：

```yaml
base_command_dims:
  vx: 0
  vy: 1
  vw: 2
  height: 3
```

则统一动作的归一化参数映射为：

```math
o^a_{32}=o^b_0,
\qquad
c^a_{32}=c^b_0,
```

```math
o^a_{33}=o^b_1,
\qquad
c^a_{33}=c^b_1,
```

```math
o^a_{34}=o^b_2,
\qquad
c^a_{34}=c^b_2,
```

```math
o^a_{41}=o^b_3,
\qquad
c^a_{41}=c^b_3.
```

如果角速度字段命名为 `vyaw`，其处理方式与 `vw` 相同。

---

## 11. 每个状态模块使用的归一化量

默认状态归一化类型为：

```yaml
state_norm_type: minmax_q
```

| 状态模块 | 统一槽位 | 是否归一化 | 统计来源 | 说明 |
|---|---|---:|---|---|
| 左末端位置 xyz | `[0:3]` | 是 | 左末端绝对状态统计的前 3 维 | 使用 `state_norm_type` |
| 左末端 rotation-6D | `[3:9]` | 否 | 无 | 保持几何表示原值 |
| 左夹爪 | `[9:10]` | 是 | 左夹爪状态统计 | 使用 `state_norm_type` |
| 左灵巧手 | `[10:16]` | 是 | 左手状态统计 | 使用 `state_norm_type` |
| 右末端位置 xyz | `[16:19]` | 是 | 右末端绝对状态统计的前 3 维 | 使用 `state_norm_type` |
| 右末端 rotation-6D | `[19:25]` | 否 | 无 | 保持几何表示原值 |
| 右夹爪 | `[25:26]` | 是 | 右夹爪状态统计 | 使用 `state_norm_type` |
| 右灵巧手 | `[26:32]` | 是 | 右手状态统计 | 使用 `state_norm_type` |
| 腰部 | `[32:35]` | 是 | 腰部状态统计 | 最多取前 3 维 |
| 躯干 | `[35:38]` | 是 | 躯干状态统计 | 最多取前 3 维 |
| 机体重力方向 $\mathbf g^B$ | `[41:44]` | 仅做单位长度归一化 | 无数据集统计量 | 应满足 $\|\mathbf g^B\|_2=1$ |
| 机体角速度 $\boldsymbol\omega^B$ | `[44:47]` | 是 | 三轴角速度状态统计 | 写入归一化结果 $\widehat{\boldsymbol\omega}^B$ |
| 左腿 | `[48:54]` | 是 | 左腿状态统计 | 最多取前 6 维 |
| 右腿 | `[54:60]` | 是 | 右腿状态统计 | 最多取前 6 维 |
| 未填充的底盘标量角速度/高度 | `[40:41]`, `[47:48]` | 否 | 无 | 数值为 0，掩码为 0 |

### 11.1 为什么末端位姿状态只归一化 xyz

原始绝对位姿可能分别采用：

- xyz + RPY：6 维；
- xyz + quaternion：7 维；
- xyz + rotation vector：6 维。

统一末端状态采用 xyz + rotation-6D：9 维。如果把原始旋转统计量直接应用到 rotation-6D，维数和几何语义都会不一致。因此只使用原始统计量的前三维：

```math
\widehat{\mathbf p}_t
=
\frac{\mathbf p_t-\mathbf o_{xyz}}
{\mathbf c_{xyz}},
```

而 rotation-6D 保持不变。

底盘状态不采用该位姿归一化规则；其 `[41:47]` 槽位按照第 4.3 节构造为重力方向和归一化角速度。

---

## 12. 同一本体机器人不同任务的统计合并

### 12.1 合并单位

统计合并只在**同一本体机器人**内部进行。每个任务的统计字典视为一个独立统计组，然后把该机器人全部任务的统计量一次性合并。

设同一本体机器人包含任务集合：

```math
\mathcal T=\{\tau_1,\tau_2,\ldots,\tau_M\}.
```

只有满足以下条件的任务才允许进入同一次合并：

- 机器人本体型号相同；
- 动作和状态模块定义相同；
- 坐标系、轴方向和单位相同；
- 同名字段的物理语义相同；
- 动作维数和维度排列相同。

该规则意味着：

- 每个任务权重相同；
- 每个任务贡献一个统计组；
- 任务样本量不会改变该任务在合并中的权重；
- 不同机器人本体的统计量不得合并；
- 不同本体即使具有相同的统一槽位，也必须分别维护 normalizer。

### 12.2 均值合并与等权假设

等组权重的准确含义不是假设不同任务的运动轨迹内容相似，而是假设训练时每个任务被选中的概率近似相同。等价的采样过程是：

1. 先以相同概率选择一个任务；
2. 再从该任务内部采样一条轨迹或一个训练样本。

设第 $i$ 个任务的动作分布为 $P_i(\mathbf x)$，则训练时对应的目标混合分布为：

```math
P_{train}(\mathbf x)
=
\frac{1}{M}
\sum_{i=1}^{M}P_i(\mathbf x).
```

在这种任务均衡采样策略下，即使不同任务包含的原始轨迹数量不同，等任务权重仍然与模型实际看到的训练分布一致。

如果训练阶段不是任务均衡采样，而是从全部原始帧中均匀采样，那么只有当每个任务贡献的有效轨迹数、轨迹长度或动作点数量近似相同时，等组权重才近似等价于样本级合并。这里要求近似的是**各任务的有效采样数量或采样概率**，而不是各任务轨迹的运动内容相似。

设共有 $M$ 个统计组，第 $i$ 组均值为 $\boldsymbol\mu_i$。在任务均衡假设下，合并均值为：

```math
\boldsymbol\mu
=
\frac{1}{M}
\sum_{i=1}^{M}\boldsymbol\mu_i.
```

### 12.3 标准差合并

设第 $i$ 组总体标准差为 $\boldsymbol\sigma_i$，则合并方差为：

```math
\boldsymbol\sigma^2
=
\frac{1}{M}
\sum_{i=1}^{M}\boldsymbol\sigma_i^2
+
\frac{1}{M}
\sum_{i=1}^{M}\boldsymbol\mu_i^2
-
\boldsymbol\mu^2.
```

也可以写成：

```math
\boldsymbol\sigma^2
=
\underbrace{
\frac{1}{M}\sum_{i=1}^{M}\boldsymbol\sigma_i^2
}_{\text{平均组内方差}}
+
\underbrace{
\frac{1}{M}\sum_{i=1}^{M}
(\boldsymbol\mu_i-\boldsymbol\mu)^2
}_{\text{组间均值方差}}.
```

最终逐元素计算：

```math
\boldsymbol\sigma
=
\sqrt{\max(\boldsymbol\sigma^2,0)}.
```

如果某个标准差没有对应均值，则退化为：

```math
\boldsymbol\sigma
=
\sqrt{
\frac{1}{M}
\sum_{i=1}^{M}\boldsymbol\sigma_i^2
}.
```

### 12.4 极值合并

```math
\mathbf x_{min}
=
\min_i\mathbf x_{min}^{(i)},
```

```math
\mathbf x_{max}
=
\max_i\mathbf x_{max}^{(i)}.
```

所有比较均逐元素进行。

### 12.5 分位数合并

当前规范使用保守包络：

```math
Q_{0.01}^{merged}
=
\min_i Q_{0.01}^{(i)},
```

```math
Q_{0.99}^{merged}
=
\max_i Q_{0.99}^{(i)}.
```

其他分位数字段采用各组对应统计值的逐元素中位数。

必须注意：上述结果不是混合样本的真实分位数。仅凭各组的少数分位点无法精确恢复混合分布分位数。若需要真实分位数，必须重新扫描原始样本，或保存可合并的直方图、t-digest 等摘要。

### 12.6 Count 合并

若普通统计量包含 `count`，合并后为：

```math
N=\sum_{i=1}^{M}n_i.
```

但本规范对应的均值和标准差仍按任务等权合并，不使用 count 加权。因此 `count` 仅作为记录信息，不影响当前归一化参数。

### 12.7 形状兼容规则

对同一字段和同一统计项，只有形状一致的数组可以合并。

- 相对动作统计：所有组应具有相同动作维数；
- 普通统计：只合并与第一个有效统计项形状一致的数组；
- 如果普通统计中少于两个数组形状兼容，则直接使用第一个有效统计项；
- 图像统计不参与动作和状态低维统计合并。

### 12.8 等组权重与样本权重的区别

当前等组权重适合强调任务平衡。如果希望统计量严格对应所有样本的总体分布，应使用样本数加权。

设第 $i$ 组样本数为 $n_i$：

```math
N=\sum_{i=1}^{M}n_i,
```

```math
\boldsymbol\mu_{weighted}
=
\frac{1}{N}
\sum_{i=1}^{M}n_i\boldsymbol\mu_i,
```

```math
\boldsymbol\sigma^2_{weighted}
=
\frac{1}{N}
\sum_{i=1}^{M}
 n_i\left(
 \boldsymbol\sigma_i^2+
 \boldsymbol\mu_i^2
 \right)
-
\boldsymbol\mu_{weighted}^2.
```

该公式仅作为另一种统计口径说明，不属于当前默认合并结果。

---

## 13. 左右手统计合并

### 13.1 合并方式

当启用左右手共享统计量时，把左手和右手视为两个等权统计组。

设左、右手均值向量分别为：

```math
\boldsymbol{\mu}_L,
\qquad
\boldsymbol{\mu}_R.
```

则合并均值为：

```math
\boldsymbol{\mu}_{LR}
=
\frac{\boldsymbol{\mu}_L+\boldsymbol{\mu}_R}{2}.
```

左右手合并方差为：

```math
\boldsymbol{\sigma}_{LR}^2
=
\frac{\boldsymbol{\sigma}_L^2+
      \boldsymbol{\sigma}_R^2}{2}
+
\frac{\boldsymbol{\mu}_L^2+
      \boldsymbol{\mu}_R^2}{2}
-
\boldsymbol{\mu}_{LR}^2.
```

分位数范围为：

```math
Q_{0.01}^{LR}
=
\min(Q_{0.01}^{L},Q_{0.01}^{R}),
```

```math
Q_{0.99}^{LR}
=
\max(Q_{0.99}^{L},Q_{0.99}^{R}).
```

合并后，左手和右手必须使用完全相同的 offset 和 scale。

### 13.2 合并顺序

正确顺序为：

1. 计算每个任务的左右手统计量；
2. 合并同一本体机器人下的全部任务；
3. 对已经合并好的左手统计和右手统计再执行左右手合并；
4. 将同一份最终统计量赋给左手和右手。

即：

```math
\text{任务级统计}
\longrightarrow
\text{跨任务/变体合并}
\longrightarrow
\text{左右手合并}
\longrightarrow
\text{normalizer}.
```

### 13.3 坐标一致性前提

左右手合并不会自动执行镜像、换轴或符号翻转。合并前必须保证：

- 左右手相对平移的 xyz 轴具有相同语义；
- 左右手旋转向量采用相同右手系；
- 左右手相同索引表示相同物理方向；
- 单位相同。

如果右手坐标需要镜像到左手规范坐标，必须先定义固定变换：

```math
\widetilde{\mathbf a}_R=M\mathbf a_R,
```

再使用 $\widetilde{\mathbf a}_R$ 计算右手统计量。矩阵 $M$ 应同时处理平移和旋转向量的轴交换与符号变化。

### 13.4 合并目的

共享左右手 normalizer 可以：

1. 消除左右手训练数据量不同造成的尺度差异；
2. 保证左右手交换或对称增强前后的数值空间一致；
3. 使共享动作头学习统一的双臂运动先验。

---

## 14. 夹爪二值化

夹爪二值化在归一化之后执行。设归一化夹爪序列为：

```math
\widehat{g}_0,\widehat{g}_1,\ldots,
\widehat{g}_{H-1}.
```

明确状态定义为：

```math
\widehat{g}_k>0.9
\quad\Longrightarrow\quad
b_k=1,
```

```math
\widehat{g}_k<-0.9
\quad\Longrightarrow\quad
b_k=0.
```

处于区间 $[-0.9,0.9]$ 的值属于中间状态。处理时从序列末尾向前扫描，并使用后续最近的明确状态回填。

尾部初始类别为：

```math
b_{H-1}^{init}
=
\begin{cases}
1,&\widehat{g}_{H-1}>0,\\
0,&\widehat{g}_{H-1}\le 0.
\end{cases}
```

伪代码如下：

```text
carry = 1 if normalized_gripper[-1] > 0 else 0

for k from H-1 down to 0:
    if normalized_gripper[k] > 0.9:
        carry = 1
    else if normalized_gripper[k] < -0.9:
        carry = 0

    binary_gripper[k] = carry
```

二值化输出属于 $\{0,1\}$，不再是普通的对称归一化连续量。

---

## 15. 动作序列重采样

### 15.1 线性重采样

若源动作块有 $N_s$ 个点，源时间戳为：

```math
t_i^{src}=
\frac{i}{f_s},
\qquad i=0,\ldots,N_s-1.
```

目标时间戳为：

```math
t_j^{tgt}=
\frac{j}{f_t},
\qquad j=0,\ldots,f_t-1.
```

每个动作维度独立做分段线性插值。若目标时间超出源时间范围，则使用最近端点值，不进行线性外推。

默认一秒线性模式读取 $N_s=f_s$ 个点，时间范围为：

```math
\left[0,\frac{f_s-1}{f_s}\right].
```

### 15.2 B-spline 重采样

平滑重采样模式读取 2 秒上下文，包括 $t=0$ 锚点：

```math
N_s=2f_s+1.
```

源时间为：

```math
t_i^{src}=\frac{i}{f_s},
\qquad i=0,\ldots,2f_s.
```

目标仍为第一个 1 秒窗口：

```math
t_j^{tgt}=\frac{j}{f_t},
\qquad j=0,\ldots,f_t-1.
```

使用三次均匀 B-spline，阶数为 3，基函数数量为：

```math
K=\max\left(4,\left\lfloor\frac{N_s}{2}\right\rfloor+1\right).
```

设源时间和目标时间上的基函数矩阵分别为 $B_{src}$ 和 $B_{tgt}$，正则项为：

```math
\lambda=10^{-9}.
```

预计算重采样矩阵：

```math
W=
B_{tgt}
\left(
B_{src}^{\mathsf T}B_{src}+\lambda I
\right)^{-1}
B_{src}^{\mathsf T}.
```

对任意动作维度，重采样结果为：

```math
A_{tgt}=WA_{src}.
```

为了逐位复现，所有实现必须采用相同的均匀 B-spline 节点定义、边界条件、基函数排列和浮点精度。

### 15.3 执行顺序

动作处理顺序固定为：

```math
\text{相对化}
\longrightarrow
\text{归一化}
\longrightarrow
\text{夹爪二值化（可选）}
\longrightarrow
\text{重采样}
\longrightarrow
\text{统一槽位映射}.
```

状态只取当前帧，不进行时间重采样。

---

## 16. 关键复现约束总结

复现本数据处理流程时，必须满足：

1. 动作固定为 54 维，状态固定为 60 维；
2. 末端状态使用绝对 xyz + rotation-6D；
3. rotation-6D 使用旋转矩阵前两列；
4. 末端和底盘位姿动作使用 $T_t^{-1}T_{t+k}$；
5. 相对位姿输出使用 xyz + rotation vector；
6. 相对动作只统计并使用展平样本维和时间维后的全局统计量；
7. 相对动作 normalizer 使用 `global_*` 统计量；
8. 相对位姿默认使用 Z-score；
9. 其他动作和状态默认使用 q01/q99 min-max；
10. EE 位姿状态只归一化 xyz，不归一化 rotation-6D；
11. 底盘状态 `[41:47]` 使用机体重力方向和归一化三轴角速度；
12. 灵巧手动作使用夹爪归一化类型；
13. 统计量只在同一本体机器人的不同任务之间合并，并采用任务等权；
14. q01/q99 合并采用范围包络，而不是真实混合分位数；
15. 左右手合并后必须共享完全相同的 offset 和 scale；
16. 左右手统计合并前必须保证坐标语义一致；
17. 无效槽位保持数值 0、offset 0、scale 1，并通过 mask 排除。
