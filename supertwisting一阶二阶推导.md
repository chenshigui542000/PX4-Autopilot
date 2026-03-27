## 滑膜控制

### 动力学

考虑带扰动的一阶滑模动力学：
$$
\dot{x}=u+\Delta(t),
$$
**状态变量** x(t) 来表示**系统的输出或状态**，比如：

- 位置（机械系统中）
- 速度（当位置是积分的结果）
- 温度、电压等（在物理系统中）

而 u(t) 是**控制输入（control input）**，也就是我们能够“施加”的信号，用来控制系统的行为。

控制系统研究的是 **输入如何影响状态随时间的演化**，所以必须涉及导数。

控制器通过计算（基于误差、规则、算法）决定 u 的取值， 然后“施加”到系统上。



我们希望系统的状态 x(t) 跟踪目标值 $x_d(t)$。

定义跟踪误差：
$$
e= x - x_d
$$
我们的目标是：
$$
e(t) \to 0 \quad \text{（状态跟踪目标）}
$$

### 定义滑模面

滑模控制的关键是构造一个**“滑模面”**（sliding surface）：
$$
s=ce=c(x−x_d)
$$
其中 c>0 是一个常数（**调节收敛速度**的参数）。
**目标：** 让系统进入并保持在 s=0 上。

当 s=0 时，就意味着 $x=x_d$​，系统完美跟踪。

### 滑模面的动态

代入系统方程，对 s 求导：
$$
\dot{s} = c \dot{e} = c(\dot{x} - \dot{x}_d)
$$
代入系统模型 $\dot{x} = u + \Delta(t)$​：
$$
\dot{s} = c(u + \Delta(t) - \dot{x}_d)
$$

### 设计控制律 u

设计思路：已知补偿 + 不确定项抑制

滑模控制把 u 拆成两部分：
$$
u=u_{eq}+u_{sw}
$$
拆成两部分，$u=u_{eq}+u_{sw}$​​，解释一下

一个负责保持平衡，一个处理扰动

① 等效控制（Equivalent Control） **$u_{eq}$​**

想让系统在滑模面上**保持**（即 $\dot{s} = 0$），忽略扰动时，
$$
\dot{s} = c(u_{eq} - \dot{x}_d) = 0
$$
所以：
$$
u_{eq} = \dot{x}_d
$$
它是**理想情况下**让系统保持在滑模面的“等效输入”。

② 切换控制（Switching Control） $u_{sw}$

现实中有扰动 Δ(t)，系统会偏离滑模面。我们要设计一个强有力的控制项，把系统**拉回滑模面**。

因此加上：
$$
u_{sw} = -k \, \text{sign}(s)
$$
其中 $ k > |\Delta(t)|_{max}$​，即控制增益必须大于最大扰动。为什么不是

注意，我们的目标是让 s = 0，当有正扰动时候，s > 0 ，加一个 $-k$​ 进行抵消；负扰动同理

就像在“敲打”系统两边，始终把它推回中线。 这就是“滑动”沿着面运动的意思。

这样：
$$
u = u_{eq} + u_{sw} = \dot{x}_d - k \, \text{sign}(s)
$$


### 验证滑模条件

将（11）代入（6） $\dot{s} = c(u + \Delta - \dot{x}_d))$​：
$$
\dot{s}=c(−ksign(s)+Δ)
$$
取 Lyapunov 函数 $ V = \frac{1}{2}s^2$，

则
$$
\dot { V } = s \dot { s } = c s ( - k \operatorname { s i g n } ( s ) + \Delta )
$$
注意到 $s⋅sign(s)=∣s∣$​，所以：
$$
\dot{V} = -c k |s| + c s \Delta 
$$


只要$ k>∣Δ∣max$，就有：
$$
\dot{V} < 0
$$
这说明系统状态会被“吸附”到滑模面 s=0，并保持在上面。

这其实是**Lyapunov稳定性定理**的核心思想：

> 若存在一个正定函数 V(s)，且其导数 $\dot{V}(s)$ 是负定的（即总在变小），
>  那么系统状态会不断趋近于使 V=0 的点，也就是稳定点。



## Supertwisting 

### 形式

传统滑模控制（sign函数型）是：
$$
u = -k \, \text{sgn}(s)
$$
它能保证 s→0 但由于符号函数的**不连续性**，会造成严重的 **抖振（chattering）**。

为了**消除抖振**，Levant（1993, 1998）提出了 **Supertwisting 算法（STA）**。

该算法引入一个内部动态变量 v，控制律如下：
$$
\boxed{ \begin{aligned} u &= -k_1 |s|^{1/2} \operatorname{sgn}(s) + v, \\ \dot{v} &= -k_2 \operatorname{sgn}(s), \end{aligned} }
$$
其中：

- $k_1>0,\, k_2>0$为增益；
- $\operatorname{sgn}(s)$ 为符号函数；
- v 是一个“辅助状态”，用来平滑控制输入、补偿扰动。

它通过引入积分项 v，实现了**连续控制**和**有限时间滑模收敛**，在对一阶有界扰动的系统 $x' = u + \Delta(t)$​​ 上具有强鲁棒性。

### 解释

#### **1.引入 v**

设想我们仍使用符号函数，但不直接施加在 u 上，而是让它**积分一次**：
$$
\dot{v} = -k_2 \operatorname{sgn}(s)
$$
这样 v(t) 连续（符号函数的积分是连续的）。

然后让 u 由两个部分组成：
$$
u = -k_1 \phi(s) + v
$$
其中：

- $-k_1 \phi(s)$：主反馈项，用来把 s 推向 0；
- v：慢变化的补偿项，用来**抵消扰动 Δ(t)**。



#### 2. $ |s|^{1/2}\operatorname{sgn}(s)$？

这就是**Supertwisting的精髓**。我们希望 u 连续，但仍然保持**有限时间收敛**。
 为此，控制器必须满足两个要求：

1. 具有 **同质性（homogeneity）**；
2. 能保证 $s、\dot{s}$ 同时趋于 0。

**(1) 同质性思想（homogeneity）**

Levant 证明：要实现**有限时间收敛**且控制信号**连续**，

系统必须在原点附近具有**负齐次度（homogeneity degree < 0）**。

若我们写闭环系统：
$$
\begin{cases} \dot{s} = -k_1 |s|^{\alpha}\operatorname{sgn}(s) + v + \Delta,\\ \dot{v} = -k_2 \operatorname{sgn}(s), \end{cases}
$$
要求系统对某个缩放比例 $\rho$​ 满足齐次性，即
$$
s \sim \rho^p \quad v\sim \rho^q
$$

 则齐次度 $d=min⁡(αp−p,q−p)$。

为了使系统收敛且 u 连续，需要：

- 齐次度 d<0
- u 连续 ⇒ α<1

解得最合适的指数是：
$$
\boxed{\alpha = \tfrac{1}{2}}
$$
也就是说，**只有当反馈项是  $ {|s|}^{1/2}\operatorname{sgn}(s)$** 时，系统才既是连续的，又能保证有限时间收敛。

### 形成闭环系统

将控制律代入系统：
$$
\dot{s} = u + \Delta(t) = -k_1 |s|^{1/2} \operatorname{sgn}(s) + v + \Delta(t)
$$
并与辅助状态方程一起写成二阶系统：
$$
\boxed{ \begin{cases} \dot{s} = -k_1 |s|^{1/2} \operatorname{sgn}(s) + v + \Delta(t),\\[3pt] \dot{v} = -k_2 \operatorname{sgn}(s). \end{cases} }
$$

### 最终形式

$$
\boxed{ \begin{aligned} s &=   e, \\ u &=  - k_1 |s|^{1/2} \operatorname{sgn}(s) + v, \\ \dot{v} &= -k_2 \operatorname{sgn}(s) \end{aligned} }
$$

### 收敛性质

在扰动满足 $ \Delta(t)| \le \Delta_{\max}, \ |\dot{\Delta}(t)| \le L$
 的条件下，如果$ k_1、k_2$​选得足够大（与扰动上界有关），则存在有限时间 T>0，使得：
$$
{s}(t) = 0,\quad \dot{s}(t) = 0, \quad \forall t \ge T
$$
这意味着：

- 状态 s 在有限时间内到达并保持在滑模面上；
- 控制输入 u 是连续的（因为 v 连续），从而几乎**完全消除了抖振**。

## 二阶控制

### 动力学

考虑受扰的二阶单输入系统（把 x 当作跟踪误差或状态）：
$$
\ddot x = u + \Delta(t)
$$

误差：
$$
e = x - x_d,\quad \dot{e} = \dot{x} - \dot{x}_d ,\quad \ddot{e} = \ddot{x} - \ddot{x}_d
$$

### 定义滑模面

**如果仍然选 s=ce，会发生什么？**

我们算它的导数：
$$
\dot{s} = c\dot{e} =c( \dot{x} - \dot{x}_d)
$$
而系统方程是 $\ddot{x} = u$
 **控制输入 u** 出现在  $\ddot{x}$，也就是 $\dot{s}$ 的导数 $ \ddot{s} $ 里。

这意味着：

- 控制输入对 s 的影响是**间接的**；
- 不能直接让 s 收敛到 0；
- 控制律必须作用在 $ \dot{s} $ 上才能生效。

→ 所以 s = e 对二阶系统 **不合适**。

---

**改进：**选  $ s = \dot{e} + \lambda e$

计算导数：
$$
\dot{s} = \ddot{e} + \lambda \dot{e}
$$
误差：
$$
\ddot{e} = \ddot{x} - \ddot{x}_d
$$
将系统模型（26）代入误差：
$$
\ddot{e}= u +\Delta - \ddot{x}_d
$$
代入（29）得到
$$
\dot{s} = u - \ddot{x}_d + \lambda \dot{e} + \Delta
$$
现在就变成：
$$
\dot{s} = u + (\lambda \dot{e} - \ddot{x}_d + \Delta)
$$
看到了吗？
现在 **控制输入 u** 直接出现在  $\dot{s}$ 中，

于是你可以设计：
$$
u = - \lambda \dot{e}+\ddot{x}_d  -\Delta- k\,\operatorname{sgn}(s)
$$
从而：
$$
\dot{s} = -k\,\operatorname{sgn}(s)
$$
保证 s→0。

---

**为什么加上 λe？**

如果不加λe， 则滑膜变量 $s = \dot{e} $ ，到达滑模面 s=0 后，$\dot{e}=0$

**但是误差 e 本身可能不为零！**
 因为滑模面只是约束  $\dot{e}=0$，并没有对 e 施加约束。

加上λe，$\dot{s} = \ddot{e} + \lambda \dot{e}$，

到达滑模面后：$s = 0 \Rightarrow \dot{e} + \lambda e = 0$

这是一个一阶线性微分方程，解为：
$$
e(t) = e(0) e^{-\lambda t} 
$$

- 收敛速度由 λ 决定，λ 越大，误差衰减越快
- 当 $t \to \infty$，误差趋近 0：

### 滑模面动态

已知
$$
\ddot x = u + \Delta(t) \\
e = x - x_d,\quad \dot{e} = \dot{x} - \dot{x}_d ,\quad \ddot{e} = \ddot{x} - \ddot{x}_d
$$
$\Delta(t)为未知扰动，满足 |\Delta(t)| \le \Delta_{\max}∣, |\dot{\Delta}(t)| \le L$

选择**一阶滑模面**：
$$
\boxed{s = \dot{e} + \lambda e = \dot{x} - \dot{x}_d + \lambda(x - x_d)}
$$
滑模面到达后：$\dot{e} + \lambda e = 0 \Rightarrow e \to 0$ 指数收敛

---

对滑模面求导：
$$
\dot{s} = \ddot{e} + \lambda \dot{e} = \ddot{x} - \ddot{x}_d + \lambda (\dot{x} - \dot{x}_d)
$$
代入系统方程：
$$
\dot{s} = u + \Delta(t) - \ddot{x}_d + \lambda (\dot{x} - \dot{x}_d)
$$
定义“已知项”：
$$
f(t) := - \ddot{x}_d + \lambda (\dot{x} - \dot{x}_d)
$$
则滑模面动态方程为：
$$
\dot{s} = u + f(t) + \Delta(t)
$$
核心思想：把所有已知项 f(t) 放到控制律里，剩下 u 直接作用在 $\dot{s}$​​ 上。

- f(t)是“已知项”，因为  $ \dot{x}, x_d, \dot{x}_d, \ddot{x}_d $​都是可测或已知的；
- Δ(t) 是**未知扰动项**，我们不能直接补偿。



### 设计 Super-Twisting 控制律

设计思路：已知的确定性补偿 + 不确定项抑制

于是我们让控制输入 u 分成两部分：
$$
\boxed{u = u_{\text{eq}} + u_{\text{st}}}
$$
Super-Twisting 控制律（适用于相对阶 1 的滑模面）形式：
$$
\begin{cases} u_{\text{st}} = -k_1 |s|^{1/2} \operatorname{sgn}(s) + v \\[2mm] \dot{v} = -k_2 \operatorname{sgn}(s) \end{cases}
$$

> 这里 v 是一个**辅助状态**，通过积分方式实现连续控制，减少抖振。
>  $k_1,k_2 >0$​ 是增益，根据扰动界限选择。

于是定义：
$$
\boxed{u_{\text{eq}} := -f(t)} \quad \Rightarrow \quad u = -f(t) + u_{\text{st}}
$$
代回滑模动态方程：
$$
\dot{s} = (-f(t) + u_{\text{st}}) + f(t) + \Delta(t) = u_{\text{st}} + \Delta(t)
$$
现在我们得到了一个**简化的动态方程**：
$$
\boxed{\dot{s} = u_{\text{st}} + \Delta(t)}
$$
这就变成了标准的 Super-Twisting 可控形式：
$$
\begin{cases} u_{\text{st}} = -k_1 |s|^{1/2}\operatorname{sgn}(s) + v \\[2mm] \dot{v} = -k_2 \operatorname{sgn}(s) \end{cases}
$$
控制律解析：

- $-k_1 |s|^{1/2} \operatorname{sgn}(s)$ 提供有限时间收敛
- v 是积分辅助项，使 u 连续

### **最终的二阶形式**

$$
\boxed{ \begin{aligned} s &= \dot{e} + \lambda e, \\ u &= f(t)_{\text{known}} - k_1 |s|^{1/2} \operatorname{sgn}(s) + v, \\ \dot{v} &= -k_2 \operatorname{sgn}(s) \end{aligned} }
$$

其中
$$
f(t)_{\text{known}} = -\ddot{x}_d + \lambda (\dot{x} - \dot{x}_d)
$$





supertwisting证明

