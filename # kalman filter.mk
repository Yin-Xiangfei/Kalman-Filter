# Kalman Filter

用途：存在不确定信息的动态系统，卡尔曼滤波就可以对系统下一步要做什么做出有根据的推测。

### 飞行器运行
状态：Position（位置）+Velocity（速度）

$$
\vec{x}=\left[\begin{array}{l}
p \\
v
\end{array}\right]
$$

![avatar](1.png)

卡尔曼滤波假设两个变量（在我们的例子里是位置和速度）都应该是**随机**的，而且符合**高斯分布**。

![avatar](2.png)

在上图中，位置和速度是**不相关**的，这意味着我们不能从一个变量推测另一个变量。\
那么如果位置和速度相关呢？如下图所示，

![avatar](3.png)

<!-- ![avatar](4.png) -->

### 用矩阵描述问题

为了把以上关于状态的信息建模为**高斯分布**，我们还需要$k$时的两个信息：**均值**$\hat{\mathbf{x}}_{k}$（就是$\mathbf{\mu}$），**协方差矩阵**$\mathbf{P}_{k}$。（虽然还是用了位置和速度两个变量，但只要和问题相关，卡尔曼滤波可以包含**任意数量**的变量）

$$
\hat{\mathbf{x}}_{k}=\left[\begin{array}{l}
\text { position } \\
\text { velocity }
\end{array}\right]
$$

$$
\mathbf{P}_{k}=\left[\begin{array}{ll}
\Sigma_{p p} & \Sigma_{p v} \\
\Sigma_{v p} & \Sigma_{v v}
\end{array}\right]
$$

我们要通过查看当前状态（$k-1$时）来预测下一个状态（$k$时）。

![avatar](5.jpg)

我们可以用矩阵$\mathbf{F}_{k}$表示这个预测步骤：

![avatar](6.jpg)

它从原始状态中取每一点，并将其移动到新的状态。下面是个简单公式：

$$
\begin{array}{l}
p_{k}=p_{k-1}+\Delta t v_{k-1} \\
v_{k}=\quad v_{k-1}
\end{array}
$$

换成矩阵形式：

$$
\begin{aligned}
\hat{\mathbf{x}}_{k} &=\left[\begin{array}{cc}
1 & \Delta t \\
0 & 1
\end{array}\right] \hat{\mathbf{x}}_{k-1} \\
&=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1}
\end{aligned}
$$

根据，

$$
\begin{aligned}
\operatorname{Cov}(x) &=\Sigma \\
\operatorname{Cov}(\mathbf{A} x) &=\mathbf{A} \Sigma \mathbf{A}^{T}
\end{aligned}
$$

可得，

$$
\begin{array}{l}
\hat{\mathbf{x}}_{k}=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1} \\
\mathbf{P}_{k}=\mathbf{F}_{\mathbf{k}} \mathbf{P}_{k-1} \mathbf{F}_{k}^{T}
\end{array}
$$

### 外部影响

除了速度和位置，外因也会对系统造成影响。\
假设油门设置和控制命令是已知的，我们知道飞行器的预期加速度$a$。根据运动学基本定理，我们可得：

$$
\begin{array}{l}
p_{k}=p_{k-1}+\Delta t v_{k-1}+\frac{1}{2} a \Delta t^{2} \\
v_{k}=\quad v_{k-1}+a \Delta t
\end{array}
$$

把它转成矩阵形式：

$$
\begin{aligned}
\hat{\mathbf{x}}_{k} &=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1}+\left[\begin{array}{c}
\frac{\Delta t^{2}}{2} \\
\Delta t
\end{array}\right] a \\
&=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1}+\mathbf{B}_{k} \overrightarrow{\mathbf{u}_{k}}
\end{aligned}
$$

### 外部不确定性

这要求我们在每个预测步骤后再加上一些新的不确定性，来模拟所有不确定性：

![avatar](7.jpg)

如上图所示，加上外部不确定性后，$\hat\mathbf{x}_{k-1}$的每个预测状态都可能会移动到另一点，也就是蓝色的高斯分布会移动到紫色高斯分布的位置，但是具有协方差$\mathbf{Q}_{k}$ 的噪声。

![avatar](8.jpg)

这个紫色的高斯分布拥有和原分布相同的均值，但协方差不同。

![avatar](9.jpg)

我们在原式上加入$\mathbf{Q}_{k}$：

$$
\begin{array}{l}
\hat{\mathbf{x}}_{k}=\mathbf{F}_{k} \hat{\mathbf{x}}_{k-1}+\mathbf{B}_{k} \overrightarrow{\mathbf{u}_{k}} \\
\mathbf{P}_{k}=\mathbf{F}_{\mathbf{k}} \mathbf{P}_{k-1} \mathbf{F}_{k}^{T}+\mathbf{Q}_{k}
\end{array}
$$

简而言之，这里：

**新的最佳估计**是基于**原最佳估计**和**已知外部影响**校正后得到的预测。\
**新的不确定性**是基于**原不确定性**和**外部环境的不确定性**得到的预测。

现在，有了这些概念介绍，我们可以把传感器数据输入其中。

### 通过测量来细化估计值

我们可能有好几个传感器，它们一起提供有关系统状态的信息。传感器的作用不是我们关心的重点，它可以读取位置，可以读取速度，重点是，它能告诉我们关于状态的间接信息——它是状态下产生的一组读数。

![avatar](10.jpg)

请注意，读数的规模和状态的规模不一定相同，所以我们把传感器**读数矩阵**设为$\mathbf{H}_{k}$。

![avatar](11.jpg)

把这些分布转换为一般形式：

$$
\begin{aligned}
\vec{\mu}_{\text {expected }} &=\mathbf{H}_{k} \hat{\mathbf{x}}_{k} \\
\mathbf{\Sigma}_{\text {expected }} &=\mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T}
\end{aligned}
$$

卡尔曼滤波的一大优点是擅长处理传感器噪声。换句话说，由于种种因素，传感器记录的信息其实是不准的，一个状态事实上可以产生多种读数。

![avatar](12.jpg)

<!-- ![avatar](13.jpg) -->

我们将这种不确定性（即传感器噪声）的协方差设为$\mathbf{R}_{k}$，读数的分布均值设为$z_k$。\
现在我们得到了两块高斯分布，一块围绕**预测的均值**，另一块围绕**传感器读数**。

![avatar](14.jpg)

如果要生成靠谱预测，模型必须调和这两个信息。\
最简单的方法是两者**相乘**。

![avatar](15.png)

两块高斯分布相乘后，我们可以得到它们的重叠部分，这也是会出现最佳估计的区域。换个角度看，它看起来也符合高斯分布：

![avatar](16.png)

最后剩下的问题就不难解决了：我们必须有一个公式来从旧的参数中获取这些新参数！

### 高斯分布

让我们从一维看起，设方差为$\sigma^{2}$，均值为$\mu$，一个标准一维高斯钟形曲线方程如下所示：

$$
\mathcal{N}(x, \mu, \sigma)=\frac{1}{\sigma \sqrt{2 \pi}} e^{-\frac{(x-\mu)^{2}}{2 \sigma^{2}}}
$$

那么两条高斯曲线相乘呢？

![avatar](17.png)

$$
\mathcal{N}\left(x, \mu_{0}, \sigma_{0}\right) \cdot \mathcal{N}\left(x, \mu_{1}, \sigma_{1}\right) \stackrel{?}{=} \mathcal{N}\left(x, \mu^{\prime}, \sigma^{\prime}\right)
$$

把这个式子按照一维方程进行扩展，可得：

$$
\begin{aligned}
\mu^{\prime} &=\mu_{0}+\frac{\sigma_{0}^{2}\left(\mu_{1}-\mu_{0}\right)}{\sigma_{0}^{2}+\sigma_{1}^{2}} \\
\sigma^{\prime 2} &=\sigma_{0}^{2}-\frac{\sigma_{0}^{4}}{\sigma_{0}^{2}+\sigma_{1}^{2}}
\end{aligned}
$$

如果有些太复杂，我们用$\mathbf{k}$简化一下：

$$
\begin{aligned}
\mathbf{k} &=\frac{\sigma_{0}^{2}}{\sigma_{0}^{2}+\sigma_{1}^{2}} \\
\mu^{\prime} &=\mu_{0}+\mathbf{k}\left(\mu_{1}-\mu_{0}\right) \\
\sigma^{\prime 2} &=\sigma_{0}^{2}-\mathbf{k} \sigma_{0}^{2}
\end{aligned}
$$

以上是一维的内容，如果是多维空间，把这个式子转成矩阵格式：

$$
\begin{array}{l}
\mathbf{K}=\Sigma_{0}\left(\Sigma_{0}+\Sigma_{1}\right)^{-1} \\
\vec{\mu}^{\prime}=\overrightarrow{\mu_{0}}+\mathbf{K}\left(\overrightarrow{\mu_{1}}-\overrightarrow{\mu_{0}}\right) \\
\Sigma^{\prime}=\Sigma_{0}-\mathbf{K} \Sigma_{0}
\end{array}
$$

这个矩阵$\mathbf{K}$就是我们说的**卡尔曼增益**。

### 结合

截至目前，我们有用矩阵$\left(\mu_{0}, \Sigma_{0}\right)=\left(\mathbf{H}_{k} \hat{\mathbf{x}}_{k}, \mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T}\right)$预测的分布，有用传感器读数$\left(\mu_{1}, \Sigma_{1}\right)=\left(\overrightarrow{\mathbf{z}_{k}}, \mathbf{R}_{k}\right)$预测的分布。把它们代入上节的矩阵等式中：

$$\begin{aligned} \mathbf{H}_{k} \hat{\mathbf{x}}_{k}^{\prime} &=\mathbf{H}_{k} \hat{\mathbf{x}}_{k} & &+\mathbf{K}\left(\vec{z}_{k}-\mathbf{H}_{k} \hat{\mathbf{x}}_{k}\right) \\ \mathbf{H}_{k} \mathbf{P}_{k}^{\prime} \mathbf{H}_{k}^{T} &=\mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T} & &-\mathbf{K} \mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T} \end{aligned}$$

相应的，卡尔曼增益就是：

$$
\mathbf{K}=\mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T}\left(\mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T}+\mathbf{R}_{k}\right)^{-1}
$$

考虑到$\mathbf{K}$里还包含着一个$\mathbf{H}_{k}$，我们再精简一下上式：

$$\begin{aligned} \hat{\mathbf{x}}_{k}^{\prime} &=\hat{\mathbf{x}}_{k}+\mathbf{K}^{\prime}\left(\overrightarrow{\mathbf{z}_{k}}-\mathbf{H}_{k} \hat{\mathbf{x}}_{k}\right) \\ \mathbf{P}_{k}^{\prime} &=\mathbf{P}_{k}-\mathbf{K}^{\prime} \mathbf{H}_{k} \mathbf{P}_{k} \\ \mathbf{K}^{\prime}=& \mathbf{P}_{k} \mathbf{H}_{k}^{T}\left(\mathbf{H}_{k} \mathbf{P}_{k} \mathbf{H}_{k}^{T}+\mathbf{R}_{k}\right)^{-1} \end{aligned}$$

最后， $\hat{\mathbf{x}}_{k}^{\prime}$是我们的最佳估计值，我们可以把它继续放进去做另一轮预测：

![avatar](18.png)