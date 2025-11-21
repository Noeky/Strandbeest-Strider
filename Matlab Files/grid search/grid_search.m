% grid_search.m
% 对选定参数在标称值附近进行网格采样并计算指标
% 默认每个参数采样 5 点：offsets = [-2 -1 0 1 2]

% 可编辑区 ----------------------------------------------------
% 基准参数向量: [link1..link11, node4_x, node4_y]
p0 = [15;50;41.5;55.8;40.1;39.3;61.9;36.7;39.4;49;65.7; -38; -7.8];
p0 = p0(:)';

% 要采样的参数索引 (1..13)。按灵敏度排名前 10：
% 1) link_6  2) node4_x  3) link_7  4) link_1  5) link_3
% 6) link_2  7) link_8  8) link_9  9) link_4  10) node4_y
idxs = [6, 12, 7, 1, 3, 2];

% offsets 和步长
offsets = [-2 -1 0 1 2]; % 步长为 1

% top_k 配置（保留每个指标的前 K 个结果）
top_k = 10; % 保留前 K 个

% safety check for number of combinations
k = numel(idxs);
numComb = numel(offsets)^k;
maxComb = 1e6; % arbitrary safe cap
if numComb > maxComb
    error('Requested %d combinations > %d. Reduce number of sampled parameters or offsets.', numComb, maxComb);
end

% 生成组合: 通过迭代索引按需构建组合（避免一次性生成巨大矩阵）
N = numComb;
results(N) = struct('params',[],'indices',[],'metrics',[],'ok',[]);

% theta vector for simulation (coarse by default)
theta_vec = deg2rad(0:10:350);

cnt = 0;
for i = 1:N
    % build offsets for this combination
    offs = zeros(1,13);
    % compute k-digit base-(#offsets) index
    idx = i-1;
    for j = 1:k
        base = numel(offsets);
        digit = mod(idx, base) + 1;
        idx = floor(idx / base);
        offs(idxs(j)) = offsets(digit);
    end

    p = p0 + offs;
    [end_traj, ok] = simulate_leg(p, theta_vec);
    metrics = compute_metrics(end_traj);

    cnt = cnt + 1;
    results(cnt).params = p(idxs);
    results(cnt).indices = idxs;
    results(cnt).metrics = metrics;
    results(cnt).ok = ok;

    % minimal progress update every 1000 iters
    if mod(cnt,1000)==0
        fprintf('Processed %d / %d\n', cnt, N);
    end
end

% Also write a compact CSV summary (easy to open in Excel)
csvfn = fullfile(pwd,'results.csv');
fid = fopen(csvfn,'w');
if fid ~= -1
    % header: index, then param indices, then metrics
    fprintf(fid,'idx,');
    for j = 1:length(idxs)
        fprintf(fid,'p%d,', idxs(j));
    end
    fprintf(fid,'ok,step_height,step_length,stable_pairs,longest_streak,mean_y,std_y\n');
    for i = 1:N
        r = results(i);
        fprintf(fid,'%d,', i);
        for j = 1:length(idxs)
            fprintf(fid,'%.6g,', r.params(j));
        end
        fprintf(fid,'%d,%.6g,%.6g,%d,%d,%.6g,%.6g\n', r.ok, r.metrics.step_height, r.metrics.step_length, r.metrics.stable_pairs, r.metrics.longest_streak, r.metrics.mean_y, r.metrics.std_y);
    end
    fclose(fid);
    fprintf('CSV summary saved to %s\n', csvfn);
else
    warning('Could not write CSV summary to %s', csvfn);
end
fprintf('Grid search complete: %d combinations processed. No .mat saved; CSV saved to %s\n', N, csvfn);

% --- 为每个指标分别选择并保存前 top_k 的结果 ---
% 收集所有指标数据（列顺序与下面 metrics_names 对应）
metrics_names = {'step_height','step_length','stable_pairs','longest_streak','mean_y','std_y'};
vals = nan(N, numel(metrics_names)); okv = false(N,1);
for i = 1:N
    r = results(i);
    okv(i) = logical(r.ok);
    vals(i,1) = r.metrics.step_height;
    vals(i,2) = r.metrics.step_length;
    vals(i,3) = r.metrics.stable_pairs;
    vals(i,4) = r.metrics.longest_streak;
    vals(i,5) = r.metrics.mean_y;
    vals(i,6) = r.metrics.std_y;
end

valid_idx = find(okv);
if isempty(valid_idx)
    fprintf('No successful simulations to select best results.\n');
else
    for m = 1:numel(metrics_names)
        col = m;
        [~, order] = sort(vals(valid_idx, col), 'descend');
        sel = valid_idx(order);
        ksel = sel(1:min(top_k, numel(sel)));

        topfn = fullfile(pwd, sprintf('grid_search_top_%s.csv', metrics_names{m}));
        fid2 = fopen(topfn, 'w');
        if fid2 ~= -1
            fprintf(fid2, 'rank,idx,');
            for j = 1:length(idxs)
                fprintf(fid2, 'p%d,', idxs(j));
            end
            fprintf(fid2, 'ok,step_height,step_length,stable_pairs,longest_streak,mean_y,std_y\n');
            for rnk = 1:length(ksel)
                i = ksel(rnk);
                rr = results(i);
                fprintf(fid2, '%d,%d,', rnk, i);
                for j = 1:length(idxs)
                    fprintf(fid2, '%.6g,', rr.params(j));
                end
                fprintf(fid2, '%d,%.6g,%.6g,%d,%.6g,%.6g,%.6g\n', rr.ok, rr.metrics.step_height, rr.metrics.step_length, rr.metrics.stable_pairs, rr.metrics.longest_streak, rr.metrics.mean_y, rr.metrics.std_y);
            end
            fclose(fid2);
            fprintf('Top %d results by %s saved to %s\n', min(top_k,numel(sel)), metrics_names{m}, topfn);
        else
            warning('Could not write top-results CSV to %s', topfn);
        end
    end
end

function metrics = compute_metrics(end_traj)
    if isempty(end_traj)
        metrics = struct('step_height',NaN,'step_length',NaN,'stable_pairs',0,'longest_streak',0,'mean_y',NaN,'std_y',NaN);
        return;
    end
    ys = end_traj(:,2); xs = end_traj(:,1);
    metrics.step_height = max(ys)-min(ys);
    metrics.step_length = max(xs)-min(xs);
    dy = abs(diff(ys));
    % 稳定阈值: 相邻 y 差值小于阈值被视为稳定
    stab_thresh = 0.2;
    metrics.stable_pairs = sum(dy < stab_thresh);
    % longest_streak 改为 x 方向跨度（最长的连续稳定段在 x 上的绝对差值）
    if isempty(dy) || ~any(dy < stab_thresh)
        metrics.longest_streak = 0;
    else
        mask = (dy < stab_thresh);
        maxspan = 0;
        nmask = length(mask);
        i = 1;
        while i <= nmask
            if mask(i)
                s = i;
                while i <= nmask && mask(i)
                    i = i + 1;
                end
                e = i - 1;
                % 对应点索引为 s .. (e+1)
                span = abs(xs(e+1) - xs(s));
                if span > maxspan
                    maxspan = span;
                end
            else
                i = i + 1;
            end
        end
        metrics.longest_streak = maxspan;
    end
    metrics.mean_y = mean(ys);
    metrics.std_y = std(ys);
end

% ------------------ Helper: simulate_leg (local) ------------------
function [end_traj, ok] = simulate_leg(p, theta_vec)
    if nargin < 2 || isempty(theta_vec)
        theta_vec = deg2rad(0:10:350);
    end
    link_lengths = p(1:11)';
    nodes = NaN(8,2);
    nodes(4,:) = [p(12), p(13)];
    nodes(1,:) = [0,0];
    prev_pts = NaN(8,2);
    end_traj = zeros(length(theta_vec),2);
    ok = true;

    % cache
    link1 = link_lengths(1); link2 = link_lengths(2); link3 = link_lengths(3);
    link4 = link_lengths(4); link5 = link_lengths(5); link6 = link_lengths(6);
    link7 = link_lengths(7); link8 = link_lengths(8); link9 = link_lengths(9);
    link10 = link_lengths(10); link11 = link_lengths(11);
    fixed_node4 = nodes(4,:);

    for ti = 1:length(theta_vec)
        theta = theta_vec(ti);
        pivot_pos = nodes(1,:);
        nodes(2,:) = pivot_pos + link1*[cos(theta), sin(theta)];

        pts = circle_intersections(nodes(2,:), link2, fixed_node4, link3);
        if isempty(pts), ok=false; break; end
        nodes(3,:) = choose_point(pts, prev_pts(3,:), 3);

        pts = circle_intersections(nodes(3,:), link4, fixed_node4, link5);
        if isempty(pts), ok=false; break; end
        nodes(5,:) = choose_point(pts, prev_pts(5,:), 5);

        pts = circle_intersections(nodes(2,:), link7, fixed_node4, link6);
        if isempty(pts), ok=false; break; end
        nodes(6,:) = choose_point(pts, prev_pts(6,:), 6);

        pts = circle_intersections(nodes(6,:), link8, nodes(5,:), link9);
        if isempty(pts), ok=false; break; end
        nodes(7,:) = choose_point(pts, prev_pts(7,:), 7);

        pts = circle_intersections(nodes(6,:), link10, nodes(7,:), link11);
        if isempty(pts), ok=false; break; end
        nodes(8,:) = choose_point(pts, prev_pts(8,:), 8);

        prev_pts = nodes;
        end_traj(ti,:) = nodes(8,:);
    end
    if ~ok
        end_traj = end_traj(1:ti-1,:);
    end
end

% ------------------ Helper: circle_intersections (local) ------------------
function pts = circle_intersections(c1,r1,c2,r2)
    dx = c2(1) - c1(1);
    dy = c2(2) - c1(2);
    d = hypot(dx, dy);
    tol = 1e-9;
    if d < tol
        pts = [];
        return;
    end
    if d > (r1 + r2) + tol || d < abs(r1 - r2) - tol
        pts = [];
        return;
    end
    a = (r1*r1 - r2*r2 + d*d) / (2*d);
    h2 = r1*r1 - a*a;
    invd = 1.0 / d;
    px = c1(1) + a * dx * invd;
    py = c1(2) + a * dy * invd;
    if h2 <= tol
        pts = [px, py; px, py];
        return;
    end
    h = sqrt(h2);
    rx = -dy * (h * invd);
    ry =  dx * (h * invd);
    pts = [px + rx, py + ry; px - rx, py - ry];
end

% ------------------ Helper: choose_point (local) ------------------
function sel = choose_point(pts, prev, idx)
    if isempty(pts)
        error('No intersection points found for given radii/centers.');
    end
    if any(isnan(prev))
        switch idx
            case 3
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            case 5
                if pts(1,1) <= pts(2,1), sel = pts(1,:); else sel = pts(2,:); end
            case {6,7,8}
                if pts(1,2) <= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            otherwise
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
        end
        return;
    end
    d1sq = (pts(1,1)-prev(1))^2 + (pts(1,2)-prev(2))^2;
    d2sq = (pts(2,1)-prev(1))^2 + (pts(2,2)-prev(2))^2;
    if d1sq <= d2sq
        sel = pts(1,:);
    else
        sel = pts(2,:);
    end
end