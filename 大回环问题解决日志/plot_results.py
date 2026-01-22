import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# 读取数据
try:
    df = pd.read_csv('benchmark_results.csv')
except FileNotFoundError:
    print("找不到 benchmark_results.csv，请先运行 ros2 launch 跑实验！")
    exit()

# 过滤掉失败的数据
df = df[df['Success'] == True]

# 设置画图风格
sns.set(style="whitegrid")
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# 1. 规划时间对比柱状图
sns.barplot(x='Algorithm', y='PlanningTime', data=df, ax=axes[0], palette="viridis")
axes[0].set_title('Planning Time Comparison (Lower is Better)')
axes[0].set_ylabel('Time (s)')

# 2. 轨迹长度对比柱状图
sns.barplot(x='Algorithm', y='PathLength', data=df, ax=axes[1], palette="magma")
axes[1].set_title('Path Length Comparison (Lower is Better)')
axes[1].set_ylabel('Joint Path Length (rad)')

plt.tight_layout()
plt.show()
