# 关键对比

![image-20251020183403578](https://raw.githubusercontent.com/i37532/PicGo_PictureHome/main/img/image-20251020183403578.png)

**标准 continuous-time super-twisting (STA)**（连续时间形式）通常写作：
$$
u(t)=−λ_1\sqrt{∣s(t)∣}sgn(s(t))+ν(t),\\
\dot{ν}(t)=−λ_2sgn(s(t))
$$
论文中用 $𝑥_1$作滑模量 𝑠

**论文中的改进（核心思想）**：**用隐式（implicit Euler）方法离散化 STA 并把离散控制器定义为广义方程（generalized equation）的解**，得到一个在采样/数值实现下具有更好性质的离散实现。

# 具体改进点

$$
\dot { x } _ { 1 } ( t ) = u + \varphi ( t ) , \; \; \dot { \varphi } ( t ) = \Delta ( t )
$$

## 1.显式离散（explicit Euler）→隐式离散化（implicit Euler）

$$
\left\{ \begin{array} { l l } { u _ { k } = - \lambda _ { 1 } \sqrt { | { \tilde { x } } _ { 1 , k + 1 } | } \mathrm { s g n } ( { \tilde { x } } _ { 1 , k + 1 } ) + \nu _ { k + 1 } } \\ { \nu _ { k + 1 } \in \nu _ { k } - \lambda _ { 2 } \ h \ \mathrm { s g n } ( { \tilde { x } } _ { 1 , k + 1 } ) } \end{array} \right.
$$

其中$t_{k+1}- t_k=h>0$是时间步长

我们定义$f(t_k) = f_k$

变量 $ \tilde { x  } _ { 1 , k + 1 }  $ 是中间变量，用于计算输入，

由于未知干扰， $  { x  } _ { 1 , k + 1 }  $  不可用

(3)中的输入是在 $t = t_k $且在整个时间间隔$ [t_k，t_{k+1})$​ 上应用的控制。



plant 离散化模型的选择如下：
$$
\begin{array} { r } { \left\{ \begin{array} { l l } { x _ { 1 , k + 1 } = x _ { 1 , k } + h u _ { k } + h \bar { \varphi } _ { k } } \\ { \varphi _ { k + 1 } = \varphi _ { k } + h \bar { \Delta } _ { k } , } \end{array} \right. } \end{array}
$$
其中 $ ϕ_k $ 和 $∆_k$ 是(2)中 $ϕ(t)和∆(t)$ 的离散时间对应。

精确的离散化由中值定理产生，假设$∆(·)$ 是连续的：

$\bar { \varphi } _ { k } = \varphi _ { k } + ( \tau - t _ { k } ) \Delta ( \gamma ) \mathrm { ~ a n d ~ } \bar { \Delta } _ { k } = \Delta ( s ) $​

$s , \tau \, \in \, ( t _ { k } , t _ { k + 1 } ) , \; \gamma \, \in \, ( t _ { k } , \tau ) .$


$$
\tilde { x } _ { 1 , k + 1 } = x _ { 1 , k } + h u _ { k } ,
$$
将（3）代入（5）可得：
$$
\begin{array} { r } { \left\{ \begin{array} { l l } { \tilde { x } _ { 1 , k + 1 } \! = \! x _ { 1 , k } - h \lambda _ { 1 } \sqrt { | \tilde { x } _ { 1 , k + 1 } | } \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) \! + \! h \nu _ { k + 1 } } \\ { \nu _ { k + 1 } \in \nu _ { k } - \lambda _ { 2 } \ h \ \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) , } \end{array} \right. } \end{array}
$$
将 $v_{k+1}$ 代入得到：
$$
\begin{array} { r l } & { \tilde { x } _ { 1 , k + 1 } =x _ { 1 , k } - h \lambda _ { 1 } \sqrt { | \tilde { x } _ { 1 , k + 1 } | } \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) } \\ & { \qquad \qquad \qquad + \, h \nu _ { k } - h ^ { 2 } \lambda _ { 2 } \, \, \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) . } \end{array}
$$




## 2.将离散控制器定义为广义方程的解

让我们引入变量：$ \xi _ { k + 1 } \; = \; \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) $ 。控制器如下：
$$
\left\{ \begin{array} { l l } { u _ { k } = - \lambda _ { 1 } \ \sqrt { | \tilde { x } _ { 1 , k + 1 } | } \ \xi _ { k + 1 } + \nu _ { k + 1 } } \\ { \nu _ { k + 1 } = \nu _ { k } - \lambda _ { 2 } \ h \ \xi _ { k + 1 } , } \end{array} \right.
$$
同样，引入 $g ( \tilde { x } _ { 1 , k + 1 } ) = - \lambda _ { 2 } \ h ^ { 2 } \ \mathrm { s g n } ( \tilde { x } _ { 1 , k + 1 } ) ,$









# 总结

## 离散控制律

（论文公式 (5)–(11)）

隐式离散化后的 Super-Twisting 控制律为：
$$
u_k = -\lambda_1 \sqrt{|\tilde{x}_{1,k+1}|}\,\xi_{k+1} + \nu_{k+1}, \\
\nu_{k+1} = \nu_k - h\,\lambda_2\,\xi_{k+1}, \\
\xi_{k+1} \in \operatorname{sgn}(\tilde{x}_{1,k+1}), \\
x_{1,k+1} = x_{1,k} + h\,\nu_{k+1} - h\,\lambda_1 \sqrt{|\tilde{x}_{1,k+1}|}\,\xi_{k+1}.
$$

## 广义方程形式

（论文公式 (11)）
$$
\boxed{ \xi_{k+1} \in \operatorname{Sgn}\!\left( x_{1,k} + h\,\nu_k - h^2 \lambda_2 \xi_{k+1} - h\,\lambda_1 \sqrt{|\tilde{x}_{1,k+1}|}\,\xi_{k+1} \right) }
$$
该方程定义了$ \xi_{k+1} 与 \tilde{x}_{1,k+1}$之间的隐式关系。通过求解（或使用论文提供的分段闭式表达）即可获得唯一的控制输入 $u_k$。

| 符号                | 含义                                                         |
| ------------------- | ------------------------------------------------------------ |
| $x_{1,k}$           | 滑模变量  $ s(t_k) $ 在采样时刻 $ t_k $ 的值                 |
| $\nu_k$             | 动态变量 $\nu(t_k)$                                          |
| $u_k$               | 控制输入（实际施加的控制信号）                               |
| h                   | 采样周期                                                     |
| $\xi_{k+1}$         | 取值于符号集合 $\operatorname{sgn}(\tilde{x}_{1,k+1})$，是一个集合值变量，用来保持隐式定义的一致性 |
| $\tilde{x}_{1,k+1}$ | 辅助隐式变量，与 $x_{1,k+1}$ 相近但通过广义方程定义          |

## 实际计算

论文指出，上述广义方程可以化简为三个闭式分段形式（case），这使得算法**无需数值迭代**：

定义
$$
b_k = -x_{1,k} - h\nu_k
$$
则有三种情况（论文 §III）：

1. **若 $b_k < -h^2 \lambda_2$：**
   $$
   \begin{cases}  a=h \lambda_1, \\\sqrt{|\tilde { x } _ { 1 , k + 1 } |} = \frac { - a + \sqrt { a ^ { 2 } - 4 ( b _ { k } + \lambda _ { 2 } h ^ { 2 } ) } } { 2 } ,\\
   \xi_{k+1} = 1, \\ \nu_{k+1} = \nu_k - h\lambda_2, \\ u_k = -\lambda_1 \sqrt{|\tilde{x}_{1,k+1}|} + \nu_{k+1}. \end{cases}
   $$
   
2. **若 $|b_k| \le h^2 \lambda_2$​：**
   $$
   \nu_{k+1} = -x_{1,k}/h, \quad u_k = \nu_{k+1}
   $$

3. **若 $b_k > h^2 \lambda_2$​：**
   $$
   \begin{cases}  a=h \lambda_1, \\\sqrt{|\tilde { x } _ { 1 , k + 1 } |} = \frac { - a + \sqrt { a ^ { 2 } + 4 ( b _ { k } - \lambda _ { 2 } h ^ { 2 } ) } } { 2 } ,\\\xi_{k+1} = -1, \\ \nu_{k+1} = \nu_k + h\lambda_2, \\ u_k = \lambda_1 \sqrt{|\tilde{x}_{1,k+1}|} + \nu_{k+1}. \end{cases}
   $$

这些分支使得控制律在实际数值实现中变得确定、可计算。



在论文第 II 节 **Lemma 1** 中（对应连续系统稳定性），参数需满足：
$$
\boxed{ \begin{aligned} L &\ge 0, \\ \lambda_1 &> \sqrt{4\sqrt{2}L}, \\ L &< \lambda_2 < \frac{\lambda_1^2}{2\sqrt{2}} - L, \end{aligned} }
$$


其中 L 是外部扰动上界，即 $∣Δ(t)∣≤L$​。





