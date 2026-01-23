# ISTA 与 STA 公式对比与参数可比性说明

本文依据以下文件整理：

- `BBSTA.md`（ISTA 隐式离散化思路）
- `bbsta_ista_key.md`（ISTA 关键公式与分段解）
- `supertwisting一阶二阶推导.md`（连续时间 STA 推导）

---

## 1. STA（连续时间 Super-Twisting）核心公式

连续时间 STA 一般写作：

$$
\begin{aligned}
u(t) &= -\lambda_1 \sqrt{|s(t)|}\,\operatorname{sgn}(s(t)) + \nu(t) \\
\dot{\nu}(t) &= -\lambda_2\,\operatorname{sgn}(s(t))
\end{aligned}
$$

其中：

- $s$ 为滑模变量（例如误差或其线性组合）。
- $\nu$ 为内部动态变量（积分型补偿），使控制输入连续、降低抖振。
- $\lambda_1, \lambda_2 > 0$ 为增益。

**特点：**
- 连续时间、符号函数切换。
- 在扰动有界条件下可实现有限时间收敛。
- 通过 $\nu$ 的积分项使控制输入连续。



---

## 2. ISTA（隐式离散化 STA）核心公式

ISTA 的核心思想：用隐式 Euler 离散化 STA，并把离散控制器定义为广义方程的解。

**关键变量：**
- 已知量：$x_{1,k}, \nu_k$，步长 $h$。
- 需隐式求解的中间变量：$\tilde{x}_{1,k+1}$。
- 选择变量：$\xi_{k+1} \in \operatorname{sgn}(\tilde{x}_{1,k+1})$。

**隐式离散控制律：**

$$
\begin{aligned}
u_k &= -\lambda_1\sqrt{|\tilde{x}_{1,k+1}|}\,\xi_{k+1} + \nu_{k+1} \\
\nu_{k+1} &= \nu_k - \lambda_2 h\,\xi_{k+1}
\end{aligned}
$$

并满足虚拟模型：
$$\tilde{x}_{1,k+1} = x_{1,k} + h u_k$$

ISTA 通过分段闭式解（Case 1/2/3）直接得到 $u_k$，避免数值迭代；等价地由广义方程求解 $\xi_{k+1}$。

---

## 3. 相同点（STA vs ISTA）

1. **结构一致**：
    - 都包含 $-\lambda_1\sqrt{|s|}\,\operatorname{sgn}(s)$ 与内部变量 $\nu$ 的二项结构。
    - $\nu$ 都承担扰动补偿与平滑控制的作用。
2. **滑模思想一致**：
    - 都以滑模变量 $s$ 驱动控制律，追求有限时间到达。
3. **参数意义一致**：
    - $\lambda_1$ 主要影响收敛速度；
    - $\lambda_2$ 影响 $\nu$ 的变化和抗扰强度。

---

## 4. 不同点（STA vs ISTA）

1. **时间域不同**：
    - **STA**：连续时间控制律。
    - **ISTA**：离散时间控制律，显式依赖采样周期 $h$。
2. **符号函数作用位置不同**：
    - **STA**：直接对 $s(t)$ 取符号。
    - **ISTA**：对 $\tilde{x}_{1,k+1}$ 取符号，并通过隐式关系求解。
3. **实现机制不同**：
    - **STA**：直接连续计算。
    - **ISTA**：通过广义方程与分段闭式解得到唯一控制输入。
4. **数值性质差异**：
    - ISTA 在离散实现上更稳健，能有效消除采样导致的数值抖振（Chattering）。
    - STA 若直接采用显式离散化，易引入数值不稳定性。

---

## 5. STA 与 ISTA（同一环）参数能否直接相同

**结论：不建议完全相同，但可以把 STA 的参数作为 ISTA 的起始对比基线。**

**原因与说明：**
- **离散化差异**：ISTA 使用 $\nu_{k+1}=\nu_k-h\lambda_2\xi_{k+1}$，同样的 $\lambda_2$ 实际效果随采样周期 $h$ 变化。
- **参数集合不同**：ISTA 有额外的平滑/死区/衰减参数（如 `EPS/DB/TC`、`KEEP_D`），STA 没有直接对应项。
- **实现细节不同**：ISTA 的隐式求解会改变等效动态，无法与连续 STA 完全等价。

**实践建议（速度环与角速率环均适用）：**
1. 先把 STA 的 $\lambda_1,\lambda_2$ 作为 ISTA 的初始值，并保证采样周期一致。
2. 先调 ISTA 特有参数（`EPS/DB/TC` 或 `HOV_DB/HOV_TC`）以抑制抖动。
3. 若动态差异仍明显，再小幅调整 $\lambda_1,\lambda_2$ 并记录最终参数。

---

## 6. 参数表（ISTA/STA）

### 速度环 ISTA 参数

| 参数 | 默认值 | 描述 |
| :--- | :--- | :--- |
| `MPC_VEL_ISTA_EN` | 0 | 启用开关 (0=PID, 1=ISTA) |
| `MPC_ISTA_L1_XY` | 1.0 | XY 轴 $\lambda_1$ |
| `MPC_ISTA_L2_XY` | 0.5 | XY 轴 $\lambda_2$ |
| `MPC_ISTA_L1_Z` | 1.2 | Z 轴 $\lambda_1$ |
| `MPC_ISTA_L2_Z` | 0.8 | Z 轴 $\lambda_2$ |
| `MPC_ISTA_KEEP_D` | 1 | 保留 D-term (微分项) |
| `MPC_ISTA_HOV_DB` | 0.05 | 悬停速度死区 ($m/s$) |
| `MPC_ISTA_HOV_TC` | 0.5 | 悬停时 $\nu$ 衰减时间常数 ($s$) |
| `MPC_ISTA_EPS` | 0.05 | 边界层宽度 ($m/s$, 0=关闭平滑) |

### 角速率环 ISTA 参数

| 参数 | 默认值 | 描述 |
| :--- | :--- | :--- |
| `MC_RATE_ISTA_EN` | 0 | 启用开关 (0=PID, 1=ISTA) |
| `MC_RATE_ISTA_EPS` | 0.02 | 边界层宽度 ($rad/s$, 0=关闭平滑) |
| `MC_RATE_ISTA_DB` | 0.02 | 角速率死区 ($rad/s$) |
| `MC_RATE_ISTA_TC` | 0.5 | $\nu$ 衰减时间常数 ($s$) |

### 速度环 STA 参数

| 参数 | 默认值 | 描述 |
| :--- | :--- | :--- |
| `POS_STA_C` | 1.0 | Z 轴滑模面系数 c |
| `POS_STA_MASS` | 0.0 | 质量参数（Z 轴 STA 推力项） |
| `POS_STA_AL` | 4.0 | Z 轴非线性项系数 α |
| `POS_STA_LA` | 2.0 | Z 轴扰动项系数 λ |
| `POS_STA_W_LIM` | 9.8 | Z 轴 w 积分限幅 |
| `POS_STA_NMAX` | 1.0 | Z 轴 ita 归一化上限 |
| `POS_STA_NMIN` | 0.9 | Z 轴 ita 归一化下限 |
| `POS_X_STA_NMAX` | 1.0 | X 轴速度误差归一化上限 |
| `POS_X_STA_NMIN` | 0.9 | X 轴速度误差归一化下限 |
| `POS_Y_STA_NMAX` | 1.0 | Y 轴速度误差归一化上限 |
| `POS_Y_STA_NMIN` | 0.9 | Y 轴速度误差归一化下限 |

### 角速率环 STA 参数

| 参数 | 默认值 | 描述 |
| :--- | :--- | :--- |
| `ATT_STA_R_C` | 0.0 | Roll 轴滑模面系数 c |
| `ATT_STA_P_C` | 0.0 | Pitch 轴滑模面系数 c |
| `ATT_STA_Y_C` | 0.0 | Yaw 轴滑模面系数 c |
| `ATT_STA_R_AL` | 0.15 | Roll 轴非线性项系数 α |
| `ATT_STA_P_AL` | 0.15 | Pitch 轴非线性项系数 α |
| `ATT_STA_Y_AL` | 0.20 | Yaw 轴非线性项系数 α |
| `ATT_STA_R_LA` | 0.20 | Roll 轴扰动项系数 λ |
| `ATT_STA_P_LA` | 0.20 | Pitch 轴扰动项系数 λ |
| `ATT_STA_Y_LA` | 0.10 | Yaw 轴扰动项系数 λ |
| `ATT_STA_R_LIM` | 0.30 | Roll 轴 w 积分限幅 |
| `ATT_STA_P_LIM` | 0.30 | Pitch 轴 w 积分限幅 |
| `ATT_STA_Y_LIM` | 0.30 | Yaw 轴 w 积分限幅 |
| `ATT_STA_R_I` | 0.0030 | Roll 轴惯量 I |
| `ATT_STA_P_I` | 0.0030 | Pitch 轴惯量 I |
| `ATT_STA_Y_I` | 0.0 | Yaw 轴惯量 I |
| `ATT_STA_R_NMAX` | 1.0 | Roll 轴 ita 归一化上限 |
| `ATT_STA_P_NMAX` | 1.0 | Pitch 轴 ita 归一化上限 |
| `ATT_STA_Y_NMAX` | 1.0 | Yaw 轴 ita 归一化上限 |
| `ATT_STA_R_NMIN` | 0.9 | Roll 轴 ita 归一化下限 |
| `ATT_STA_P_NMIN` | 0.9 | Pitch 轴 ita 归一化下限 |
| `ATT_STA_Y_NMIN` | 0.9 | Yaw 轴 ita 归一化下限 |

---

## 7. 小结

- **STA** 是理论基石，公式简洁但离散化困难。
- **ISTA** 是工程实现的优选，通过隐式离散化彻底解决了离散实现的数值抖振问题。
- 速度环与角速率环参数应独立调优，不宜直接套用数值。
