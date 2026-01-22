import re
import csv
import os
import sys

# === 配置区域 ===
# 默认读取的日志文件名 (你可以修改这里，或者运行脚本时通过参数传入)
DEFAULT_LOG_FILE = "experiment_log.txt"
# 结果保存的 CSV 文件名
OUTPUT_CSV_FILE = "all_benchmark_data_constrainerd.csv"

def extract_from_log(log_filename):
    """
    从日志文件中提取算法、目标点、耗时和长度信息
    """
    if not os.path.exists(log_filename):
        print(f"❌ 错误: 找不到日志文件 '{log_filename}'")
        return []

    print(f"正在读取日志文件: {log_filename} ...")
    
    # 正则表达式匹配模式
    # 匹配: ********** 测试算法: RRTConnect **********
    algo_pattern = re.compile(r"测试算法:\s*([A-Za-z0-9_]+)")
    
    # 匹配: --- 前往目标点 [1 / 6] ---
    target_pattern = re.compile(r"前往目标点\s*\[(\d+)\s*/\s*(\d+)\]")
    
    # 匹配: 规划成功 | 耗时: 6.0597e-05s | 长度: 0.4475
    # 注意：[\d.eE-]+ 用于同时匹配普通小数和科学计数法
    result_pattern = re.compile(r"规划成功\s*\|\s*耗时:\s*([\d.eE-]+)s\s*\|\s*长度:\s*([\d.eE-]+)")

    data_list = []
    current_algo = "Unknown"
    current_target = -1

    with open(log_filename, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            
            # 1. 检测算法名称
            algo_match = algo_pattern.search(line)
            if algo_match:
                current_algo = algo_match.group(1)
                continue

            # 2. 检测目标点序号
            target_match = target_pattern.search(line)
            if target_match:
                current_target = int(target_match.group(1))
                continue

            # 3. 检测规划结果
            result_match = result_pattern.search(line)
            if result_match:
                try:
                    time_val = float(result_match.group(1))
                    path_len = float(result_match.group(2))
                    
                    # 将提取到的数据存入列表
                    data_list.append([current_algo, current_target, time_val, path_len])
                    # print(f"  -> 提取到: {current_algo} (点{current_target}): {time_val}s, {path_len}m")
                except ValueError:
                    print(f"⚠️ 警告: 无法解析数值: {line}")

    return data_list

def save_to_csv(data, output_file):
    """
    将数据追加写入 CSV 文件
    """
    if not data:
        print("⚠️ 未提取到任何有效数据，跳过保存。")
        return

    # 检查文件是否存在，如果不存在则需要写入表头
    file_exists = os.path.exists(output_file)
    
    try:
        with open(output_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # 如果是新文件，先写入表头
            if not file_exists:
                writer.writerow(["Algorithm", "TargetID", "PlanningTime", "PathLength"])
            
            # 写入数据行
            writer.writerows(data)
            
        print(f"✅ 成功! 已将 {len(data)} 条数据追加保存到 '{output_file}'")
    except Exception as e:
        print(f"❌ 保存 CSV 失败: {e}")

if __name__ == "__main__":
    # 获取日志文件名 (优先使用命令行参数)
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_file = DEFAULT_LOG_FILE

    # 执行提取
    extracted_data = extract_from_log(log_file)
    
    # 执行保存
    save_to_csv(extracted_data, OUTPUT_CSV_FILE)