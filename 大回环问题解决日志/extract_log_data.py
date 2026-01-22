import os
import re
import csv
import glob

# ================= 配置区域 =================
# 日志文件所在的文件夹路径
INPUT_DIR = '/home/wrx/FR15_01/'
# 匹配日志文件名的模式 (例如 run01.txt, run_test.txt 等)
FILE_PATTERN = 'run*.txt'
# 结果保存路径
OUTPUT_CSV = '/home/wrx/FR15_01/all_benchmark_data_final.csv'
# ===========================================

def parse_log_files():
    # 获取所有匹配的日志文件并排序
    search_path = os.path.join(INPUT_DIR, FILE_PATTERN)
    log_files = sorted(glob.glob(search_path))

    if not log_files:
        print(f"警告: 在 {INPUT_DIR} 下没有找到匹配 {FILE_PATTERN} 的日志文件。")
        return

    print(f"找到 {len(log_files)} 个日志文件，开始处理...")

    # 准备 CSV 数据容器
    # 表头: 日志文件名, 算法名称, 目标点ID, 规划时间, 路径长度, 是否规划成功
    csv_rows = [['LogFile', 'Algorithm', 'TargetID', 'PlanningTime', 'PathLength', 'PlanningSuccess']]

    # 预编译正则表达式，提高效率
    # 匹配算法: ********** 测试算法: RRTConnect **********
    regex_algo = re.compile(r'\*{10} 测试算法: (\w+) \*{10}')
    
    # 匹配目标点: --- 前往目标点 [1 / 6] ---
    regex_target = re.compile(r'--- 前往目标点 \[(\d+) / \d+\] ---')
    
    # 匹配成功数据: 规划成功 | 耗时: 0.0228 | 长度: 2.2763
    regex_success = re.compile(r'规划成功 \| 耗时: ([\d.]+) \| 长度: ([\d.]+)')
    
    # 匹配失败标记 (根据C++代码逻辑)
    regex_fail = re.compile(r'规划失败')

    for file_path in log_files:
        file_name = os.path.basename(file_path)
        print(f"正在解析: {file_name}")
        
        current_algo = "Unknown"
        current_target = "Unknown"
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    
                    # 1. 提取算法名称
                    match_algo = regex_algo.search(line)
                    if match_algo:
                        current_algo = match_algo.group(1)
                        continue

                    # 2. 提取目标点 ID
                    match_target = regex_target.search(line)
                    if match_target:
                        current_target = match_target.group(1)
                        continue

                    # 3. 提取规划成功的数据
                    match_success = regex_success.search(line)
                    if match_success:
                        time_val = match_success.group(1)
                        len_val = match_success.group(2)
                        csv_rows.append([
                            file_name, 
                            current_algo, 
                            current_target, 
                            time_val, 
                            len_val, 
                            'True'
                        ])
                        continue

                    # 4. 检查是否记录了规划失败
                    # 注意：如果日志中只打印错误信息而没有时间和长度，我们记为 0
                    if regex_fail.search(line):
                        csv_rows.append([
                            file_name, 
                            current_algo, 
                            current_target, 
                            0, 
                            0, 
                            'False'
                        ])

        except Exception as e:
            print(f"读取文件 {file_name} 时出错: {e}")

    # 写入 CSV 文件
    try:
        with open(OUTPUT_CSV, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(csv_rows)
        print(f"\n成功! 数据已提取并保存至: {OUTPUT_CSV}")
        print(f"共提取 {len(csv_rows)-1} 条数据记录。")
    except Exception as e:
        print(f"保存 CSV 文件时出错: {e}")

if __name__ == "__main__":
    parse_log_files()