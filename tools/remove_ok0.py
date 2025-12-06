"""tools/remove_ok0.py

Usage examples (from repository root, cmd.exe):
# write filtered output to new file
python tools\remove_ok0.py --in "Matlab Files\grid search\better_than_stats.csv" --out "Matlab Files\grid search\better_than_stats_ok1.csv"

# overwrite the original file (creates a .bak backup)
python tools\remove_ok0.py --in "Matlab Files\grid search\better_than_stats.csv" --inplace

This script:
- detects the header and finds the `ok` column (case-insensitive match of column name 'ok');
- filters out rows where ok==0 (treats numeric-like values; '0' or '0.0' or numeric 0 counts as 0);
- preserves the header if present; writes filtered rows to output.
"""
import argparse
import csv
import os
import sys


def find_ok_index(header):
    for i, h in enumerate(header):
        if h is None:
            continue
        if h.strip().lower() == 'ok':
            return i
    # fallback: try contains
    for i, h in enumerate(header):
        if h and 'ok' == h.strip().lower():
            return i
    return None


def is_zero_value(val):
    if val is None:
        return False
    s = str(val).strip()
    if s == '':
        return False
    try:
        f = float(s)
        return abs(f) < 1e-9
    except Exception:
        # also treat string '0' explicitly
        return s == '0'


def filter_file(in_path, out_path, inplace=False, backup=True):
    if not os.path.exists(in_path):
        print(f"Input file not found: {in_path}")
        sys.exit(2)

    with open(in_path, newline='', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        print('Input CSV is empty')
        return 0

    header = rows[0]
    ok_idx = find_ok_index(header)
    start_row = 1

    # If first row doesn't look like header (contains numeric values), try to detect by scanning
    if ok_idx is None:
        # scan first 10 rows for a column that looks like ok (values 0 or 1)
        nscan = min(10, len(rows)-1)
        col_scores = []
        ncols = max(len(r) for r in rows)
        for c in range(ncols):
            score = 0
            count = 0
            for r in rows[0:nscan+1]:
                if c < len(r):
                    v = r[c].strip()
                    try:
                        f = float(v)
                        if abs(f - 0) < 1e-9 or abs(f - 1) < 1e-9:
                            score += 1
                    except Exception:
                        pass
                    count += 1
            col_scores.append((score, c))
        # pick column with highest score if high enough
        col_scores.sort(reverse=True)
        if col_scores and col_scores[0][0] >= max(1, nscan//2):
            ok_idx = col_scores[0][1]
            # no header
            header = None
            start_row = 0

    filtered = []
    kept = 0
    removed = 0
    # If header present and ok_idx within header len, start after header
    for i, row in enumerate(rows[start_row:], start=start_row+1):
        if ok_idx is None or ok_idx >= len(row):
            # cannot decide, keep the row
            filtered.append(row)
            kept += 1
            continue
        v = row[ok_idx]
        if is_zero_value(v):
            removed += 1
            continue
        filtered.append(row)
        kept += 1

    # write output
    if inplace:
        bak = in_path + '.bak'
        if backup:
            try:
                os.replace(in_path, bak)
            except Exception:
                # fallback copy
                import shutil
                shutil.copy2(in_path, bak)
        out_write = in_path
    else:
        out_write = out_path

    with open(out_write, 'w', newline='', encoding='utf-8') as outf:
        writer = csv.writer(outf)
        if header is not None:
            writer.writerow(header)
        for r in filtered:
            writer.writerow(r)

    print(f'Wrote {kept} rows (removed {removed}) to {out_write}')
    return removed


def main():
    p = argparse.ArgumentParser(description='Remove rows with ok==0 from a CSV file')
    p.add_argument('--in', dest='infile', required=True, help='input CSV path')
    p.add_argument('--out', dest='outfile', help='output CSV path (ignored if --inplace)')
    p.add_argument('--inplace', dest='inplace', action='store_true', help='overwrite input file (creates .bak)')
    p.add_argument('--no-backup', dest='nobak', action='store_true', help="don't create .bak when --inplace")
    args = p.parse_args()

    outfile = args.outfile
    if not args.inplace and not outfile:
        print('Either --out or --inplace must be provided')
        sys.exit(2)

    removed = filter_file(args.infile, outfile, inplace=args.inplace, backup=not args.nobak)

if __name__ == '__main__':
    main()
