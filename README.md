# Strandbeest-Strider 仿生足连杆机构优化

本项目致力于研究和优化基于[泰奥·扬森 (Theo Jansen)](https://en.wikipedia.org/wiki/Theo_Jansen) 连杆机构的仿生足行走方案。通过几何仿真、网格搜索 (Grid Search) 和数据驱动的机器学习算法，我们探索了该机构的运动学特性，并针对步高、步长和稳定性等关键指标进行了参数优化。

## 优化策略

为了找到比原始参数更优的连杆比例，我们采用了两种主要的计算方法：

### A. 网格搜索优化 (Grid Search) - MATLAB
*路径：`Matlab Files/grid search/`*

基于几何仿真，我们建立了一套评价指标体系：
*   **关键指标**：`step_height` (步高), `step_length` (步长), `stable_pairs` (稳定对)。
*   **实施过程**：
    1.  通过单变量扰动分析，筛选出对性能影响最大的 **前8个参数**。
    2.  在 $\pm 2$ 的范围内进行采样 (offsets = [-2, -1, 0, 1, 2])。
    3.  生成并验证了 **390,625** 组参数组合，筛选出满足几何可行性的最优解。

### B. 机器学习/数据驱动优化 (Data-Driven Optimization) - Python
*路径：`Analysis/`*

在网格搜索的基础上，进一步使用 Python 脚本进行更灵活的搜索与精炼。

*   **指标体系 (Scoring System)**：
    $$score = length + w_h \cdot height - \frac{w_{spi}}{spi\_margin} - \frac{w_{asi}}{asi\_margin}$$
    综合平衡了 **稳定段长度**、**步高** 以及 **机械卡死/关节极限的安全裕度**。

*   **实施过程**：
    1.  **初始采样**：随机生成 500-600 组样本，进行几何可行性验证。
    2.  **Top-K 筛选**：根据评分排序，锁定高潜力解。
    3.  **爬山精炼**：对优选参数进行 **6轮坐标爬山 (Coordinate Hill Climbing)** 微调 (步长0.5)，输出最优解。

## 项目结构

```
Strandbeest-Strider/
├── Analysis/               # Python 分析与优化代码
│   ├── main_analysis.py    # 主分析脚本 (随机采样/爬山算法)
│   ├── leg_interface.py    # 连杆机构仿真接口
│   └── version X/          # 各版本的优化结果 (CSV/JSON)
├── Matlab Files/           # MATLAB 仿真与网格搜索代码
│   ├── leg.m               # 单腿运动学解算
│   ├── legs.m              # 多腿组合仿真
│   └── grid search/        # 网格搜索脚本及结果数据
└── requirements.txt        # Python 依赖库
```

## 快速开始

### Python 环境
安装依赖：
```bash
pip install -r requirements.txt
```

运行优化分析：
```bash
python Analysis/main_analysis.py --n-init 500 --top-k 50 --rounds 6
```
该脚本将会在 `Analysis` 目录下生成新的优化结果。

### MATLAB 环境
在 MATLAB 中打开 `Matlab Files/` 目录，运行 `leg.m` 可查看单腿仿真，或查看 `grid search/` 下的脚本进行参数搜索复现。

---
*本项目包含详细的参数演化历史，可在 `Analysis/version X/` 和 `Matlab Files/files_success/` 中查看具体的最佳参数配置。*
