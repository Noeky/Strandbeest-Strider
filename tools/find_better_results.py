"""
tools/find_better_results.py

用途：读取一个 results.csv 和一个 stats.json（基准），筛选出所有在
  - step_height 更大
  - longest_streak 更大
的行，并把这些行输出到一个新的 CSV 文件中。

用法（在 cmd.exe 中运行示例）：
python tools\find_better_results.py --results "Matlab Files\grid search\results.csv" --stats "Matlab Files\grid search\stats.json" --out "Matlab Files\grid search\better_than_stats.csv"

脚本会：
- 自动识别 CSV 的 header 行并查找包含 'step_height' 与 'longest_streak' 的列名（不区分大小写）；
- 如果没有 header，会尝试在每行中查找能解析为 float 的字段并通过列名索引（需要用户确认或手动调整）；
- 对比时使用严格大于（>）；
- 输出保留原始 header（若存在），并写入匹配的所有行。

"""
import argparse
import csv
import json
import os
import sys


def find_metric_in_stats(stats_obj, key_names):
    """递归查找 stats JSON 中第一个匹配 key_names（列表）并返回其数值。"""
    if isinstance(stats_obj, dict):
        for k, v in stats_obj.items():
            if any(kn.lower() == k.lower() for kn in key_names):
                try:
                    return float(v)
                except Exception:
                    pass
        for v in stats_obj.values():
            res = find_metric_in_stats(v, key_names)
            if res is not None:
                return res
    elif isinstance(stats_obj, list):
        for item in stats_obj:
            res = find_metric_in_stats(item, key_names)
            if res is not None:
                return res
    return None


def sniff_header_and_indices(csv_path, target_names):
    """返回 (has_header, header, idx_map)
    - has_header: True/False
    - header: list of column names if has_header else None
    - idx_map: dict mapping target_name->column_index (0-based) when found, else None entries
    """
    with open(csv_path, newline='', encoding='utf-8', errors='ignore') as f:
        sniffer = csv.Sniffer()
        sample = f.read(8192)
        f.seek(0)
        has_header = False
        try:
            has_header = sniffer.has_header(sample)
        except Exception:
            has_header = False
        reader = csv.reader(f)
        first = None
        try:
            first = next(reader)
        except StopIteration:
            return False, None, {k: None for k in target_names}

        # 如果有 header，尝试匹配列名
        idx_map = {k: None for k in target_names}
        if has_header:
            header = [c.strip() for c in first]
            for i, h in enumerate(header):
                for target in target_names:
                    if target.lower() == h.lower() or target.lower() in h.lower():
                        idx_map[target] = i
            return True, header, idx_map

        # 没有 header，则尝试在接下来的 N rows 中寻找列索引（基于列值能否解析为 float）
        # 我们退回并 read all rows
        f.seek(0)
        rows = list(csv.reader(f))
        ncols = max(len(r) for r in rows) if rows else 0
        # For each column, test whether many values parse as float; then among float columns, we cannot be sure which is which.
        # 返回 None indices so caller can fallback to searching by value sequence (if needed)
        return False, None, {k: None for k in target_names}


def read_stats(stats_path):
    if not os.path.exists(stats_path):
        print(f"stats.json not found: {stats_path}")
        sys.exit(2)
    with open(stats_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    # 尝试查找字段
    step_height = find_metric_in_stats(data, ['step_height'])
    longest_streak = find_metric_in_stats(data, ['longest_streak'])
    return step_height, longest_streak


def filter_results(results_csv, stats_vals, out_csv):
    target_names = ['step_height', 'longest_streak']
    has_header, header, idx_map = sniff_header_and_indices(results_csv, target_names)

    matched_rows = []
    total = 0
    with open(results_csv, newline='', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        print('results.csv is empty')
        return 0

    # if header present and idxs found, use them
    if has_header and header:
        for t in target_names:
            if idx_map.get(t) is None:
                # try fuzzy match
                for i, h in enumerate(header):
                    if t.lower() in h.lower():
                        idx_map[t] = i
    
    # If indices still None, try to locate columns by header-like names in first row (if not header)
    # But safest approach: if header unknown, find columns by name in header row text (if header is actually present)

    # Find positions or fallback to searching by column names in header string
    step_idx = idx_map.get('step_height')
    long_idx = idx_map.get('longest_streak')

    # If indices unknown, attempt to find by exact column names in the first row
    if step_idx is None or long_idx is None:
        first_row = rows[0]
        for i, cell in enumerate(first_row):
            if cell and isinstance(cell, str):
                cl = cell.strip().lower()
                if step_idx is None and 'step_height' in cl:
                    step_idx = i
                if long_idx is None and 'longest_streak' in cl:
                    long_idx = i

    # If still unknown, search each row for numeric fields matching stats sequence? Simpler: attempt to find columns by header names; otherwise error.
    if step_idx is None or long_idx is None:
        print('Warning: Could not auto-detect columns for step_height or longest_streak from header. Attempting best-effort by scanning rows.')

    # Try to parse rows and compare
    for lineno, row in enumerate(rows, start=1):
        total += 1
        # Skip empty
        if not any(c.strip() if isinstance(c, str) else c for c in row):
            continue
        # If header and this is the header row, preserve and continue
        if has_header and header and lineno == 1:
            continue
        try:
            # locate step and long values
            sh_val = None
            ls_val = None
            if step_idx is not None and step_idx < len(row):
                try:
                    sh_val = float(row[step_idx])
                except Exception:
                    sh_val = None
            if long_idx is not None and long_idx < len(row):
                try:
                    ls_val = float(row[long_idx])
                except Exception:
                    ls_val = None

            # fallback: try to find by column name tokens in header-like first row: sometimes CSV has header in first row
            if sh_val is None or ls_val is None:
                # attempt to locate by scanning all numeric fields in row and choosing fields by heuristics
                numeric_cols = []
                for i, cell in enumerate(row):
                    try:
                        numeric_cols.append((i, float(cell)))
                    except Exception:
                        pass
                # If we have at least two numeric columns, try to find the two that appear to be the metrics
                if sh_val is None and numeric_cols:
                    # choose maximum numeric as step_height candidate if plausible
                    sh_val = max([v for (_, v) in numeric_cols]) if numeric_cols else None
                if ls_val is None and numeric_cols:
                    # choose second metric as maybe integer-like (longest_streak often small int)
                    # choose the numeric col with smallest absolute diff to round value
                    candidates = sorted(numeric_cols, key=lambda iv: abs(iv[1] - round(iv[1])))
                    if candidates:
                        ls_val = candidates[0][1]

            if sh_val is None or ls_val is None:
                continue

            if sh_val > stats_vals[0] and ls_val > stats_vals[1]:
                matched_rows.append(row)
        except Exception:
            continue

    # write output CSV with header if available
    with open(out_csv, 'w', newline='', encoding='utf-8') as outf:
        writer = csv.writer(outf)
        if has_header and header:
            writer.writerow(header)
            for r in matched_rows:
                writer.writerow(r)
        else:
            for r in matched_rows:
                writer.writerow(r)

    print(f"Scanned {total} rows, matched {len(matched_rows)} rows. Output written to: {out_csv}")
    return len(matched_rows)


def main():
    p = argparse.ArgumentParser(description='从 results.csv 中筛选出同时在 step_height 与 longest_streak 上超过 stats.json 的行。')
    p.add_argument('--results', '-r', default=r"Matlab Files\grid search\results.csv", help='results.csv 路径')
    p.add_argument('--stats', '-s', default=r"Matlab Files\grid search\stats.json", help='stats.json 路径')
    p.add_argument('--out', '-o', default=r"Matlab Files\grid search\better_than_stats.csv", help='输出 CSV 路径')
    args = p.parse_args()

    stats_path = args.stats
    results_path = args.results
    out_path = args.out

    if not os.path.exists(results_path):
        print(f"results.csv not found: {results_path}")
        sys.exit(2)

    step_h, long_s = read_stats(stats_path)
    if step_h is None or long_s is None:
        print('无法从 stats.json 中读取 step_height 或 longest_streak，请检查文件结构。')
        sys.exit(3)

    print(f"Baseline: step_height={step_h}, longest_streak={long_s}")
    filter_results(results_path, (step_h, long_s), out_path)


if __name__ == '__main__':
    main()
