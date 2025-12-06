"""tools/sort_by_std_y.py

Sort a CSV by the `std_y` column (ascending) and write a new CSV.

Usage (cmd.exe):
python tools\sort_by_std_y.py --in "Matlab Files\grid search\better_than_default.csv" --out "Matlab Files\grid search\better_than_default_sorted.csv"

If --out is omitted, a file with suffix `_sorted_std_y.csv` will be created next to the input file.
"""
import argparse
import csv
import os
import sys


def find_column_index(header, name):
    if header is None:
        return None
    for i, h in enumerate(header):
        if h is None:
            continue
        if h.strip().lower() == name.lower():
            return i
    # fallback: contains
    for i, h in enumerate(header):
        if h and name.lower() in h.strip().lower():
            return i
    return None


def try_parse_float(s):
    try:
        return float(s)
    except Exception:
        return None


def sort_csv_by_std_y(in_path, out_path=None):
    if not os.path.exists(in_path):
        print(f"Input file not found: {in_path}")
        return 2
    if out_path is None:
        base, ext = os.path.splitext(in_path)
        out_path = base + '_sorted_std_y' + ext

    # read CSV
    with open(in_path, newline='', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        print('Empty CSV')
        return 3

    header = rows[0]
    data = rows[1:]

    std_idx = find_column_index(header, 'std_y')
    if std_idx is None:
        print('Could not find a header column named std_y (or similar). Trying to infer numeric column...')
        # try to infer column by checking which column parses as float for most rows
        ncols = max((len(r) for r in rows), default=0)
        scores = []
        for c in range(ncols):
            score = 0
            total = 0
            for r in data[:200]:
                if c < len(r):
                    if try_parse_float(r[c]) is not None:
                        score += 1
                    total += 1
            scores.append((score, c))
        scores.sort(reverse=True)
        if scores and scores[0][0] > 0:
            std_idx = scores[0][1]
            print(f'Guessing column {std_idx+1} as numeric column for sort (0-based idx {std_idx}).')
        else:
            print('Failed to infer numeric column. Aborting.')
            return 4

    # build sortable rows with parsed std values
    sortable = []
    for r in data:
        val = None
        if std_idx < len(r):
            val = try_parse_float(r[std_idx])
        sortable.append((val if val is not None else float('inf'), r))

    # sort by std (None/NaN put at end)
    sortable.sort(key=lambda t: (t[0] is None, t[0]))

    # write output
    with open(out_path, 'w', newline='', encoding='utf-8') as outf:
        writer = csv.writer(outf)
        writer.writerow(header)
        for val, r in sortable:
            writer.writerow(r)

    print(f'Wrote sorted CSV: {out_path} (rows: {len(sortable)})')
    return 0


def main():
    p = argparse.ArgumentParser(description='Sort CSV by std_y (ascending)')
    p.add_argument('--in', dest='infile', required=True, help='input CSV path')
    p.add_argument('--out', dest='outfile', help='output CSV path (optional)')
    args = p.parse_args()
    code = sort_csv_by_std_y(args.infile, args.outfile)
    sys.exit(code)

if __name__ == '__main__':
    main()
