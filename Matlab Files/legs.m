% 基础参数（可按需替换为当前优化得到的长度与 node4）
link_lengths = [15.0, 50.0, 41.5, 55.8, 40.1, 39.3, 61.9, 36.7, 39.4, 49.0, 65.7];
base_node4 = [-38.0, -7.8];           % 左腿 node4
mirror_node4 = [abs(base_node4(1)), base_node4(2)]; % 右腿 node4（关于 y 轴对称）

% 绘制模式开关：1=单组腿（2条），3=三组腿（6条）
draw_mode = 3; % 可设为 1 或 3

links_pairs = [
    1 2;
    2 3;
    3 4;
    3 5;
    4 5;
    4 6;
    2 6;
    6 7;
    5 7;
    6 8;
    7 8
];

theta_vec = linspace(0,2*pi,361); % 基准相位步进（0.5deg）

% 根据绘制模式设置相位偏移与颜色
if draw_mode == 1
    phase_offsets = deg2rad([0]); % 单组腿
    group_colors = [0.1216 0.4667 0.7059]; % 蓝色
elseif draw_mode == 3
    phase_offsets = deg2rad([0, 120, 240]); % 三组腿相位差
    group_colors = [
        0.1216 0.4667 0.7059; % 组1 蓝
        0.8510 0.3294 0.1020; % 组2 橙
        0.0980 0.6745 0.2039; % 组3 绿
    ];
else
    error('draw_mode 必须为 1 或 3');
end

n_groups = numel(phase_offsets);
legs_per_group = 2;
n_legs = n_groups * legs_per_group; % 6 条腿
n_steps = numel(theta_vec);

% 为每条腿单独维护前一帧的节点，避免选择交点时互相干扰
prev_all = NaN(n_legs, 8, 2);
nodes_history = NaN(n_steps, n_legs, 8, 2);

for ti = 1:n_steps
    for g = 1:n_groups
        phase_theta = theta_vec(ti) + phase_offsets(g);
        % 本组共享 node2（转动节点），左右腿使用同一驱动点
        shared_node2 = [cos(phase_theta), sin(phase_theta)] * link_lengths(1);

        % 两条腿：左（原始 node4）、右（镜像 node4，逻辑相反但共享 node2）
        for leg_side = 1:legs_per_group
            leg_idx = (g-1)*legs_per_group + leg_side;
            if leg_side == 1
                node4 = base_node4;
                flip_logic = false;
            else
                node4 = mirror_node4;
                flip_logic = true; % 右腿只在左右选择上取反（不反转上下）
            end
            prev_pts = squeeze(prev_all(leg_idx,:,:));
            [nodes, prev_pts] = solve_leg_pose(shared_node2, node4, link_lengths, prev_pts, flip_logic);
            prev_all(leg_idx,:,:) = prev_pts;
            nodes_history(ti, leg_idx, :, :) = nodes;
        end
    end
end

% 生成并保存机构动图（不再单独绘制末端轨迹）
script_full = mfilename('fullpath');
if isempty(script_full)
    outdir = pwd;
else
    outdir = fileparts(script_full);
end
giffile = fullfile(outdir,'leg_animation.gif');
save_png = false;
frames_dir = fullfile(outdir,'frames');
if save_png && ~exist(frames_dir,'dir')
    mkdir(frames_dir);
end

% 计算全局绘图范围
all_coords = reshape(nodes_history, [], 2);
valid = all(~isnan(all_coords),2);
all_coords = all_coords(valid,:);
if isempty(all_coords)
    error('No valid node coordinates were computed; check fixed nodes and link lengths.');
end
pad = 10;
xmin = min(all_coords(:,1)) - pad; xmax = max(all_coords(:,1)) + pad;
ymin = min(all_coords(:,2)) - pad; ymax = max(all_coords(:,2)) + pad;

fig = figure('Name','Leg Animation (3 groups)','NumberTitle','off');
for ti = 1:n_steps
    clf(fig); hold on; axis equal; grid on;
    for leg_idx = 1:n_legs
        g = ceil(leg_idx / legs_per_group);
        col = group_colors(g,:);
        curr = squeeze(nodes_history(ti, leg_idx, :, :));
        if any(isnan(curr(:)))
            continue;
        end
        for k = 1:size(links_pairs,1)
            i = links_pairs(k,1); j = links_pairs(k,2);
            Xi = curr(i,:); Xj = curr(j,:);
            plot([Xi(1) Xj(1)], [Xi(2) Xj(2)], '-', 'LineWidth', 2, 'Color', col);
            plot(Xi(1), Xi(2), 'o', 'MarkerFaceColor','w', 'MarkerEdgeColor', col);
            plot(Xj(1), Xj(2), 'o', 'MarkerFaceColor','w', 'MarkerEdgeColor', col);
        end
    end
    xlim([xmin xmax]); ylim([ymin ymax]);
    title(sprintf('Combined legs, frame %d / %d', ti, n_steps));
    drawnow;

    frame = getframe(fig);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if ti == 1
        imwrite(imind,cm,giffile,'gif','Loopcount',inf,'DelayTime',0.03);
    else
        imwrite(imind,cm,giffile,'gif','WriteMode','append','DelayTime',0.03);
    end
    if save_png
        pngfn = fullfile(frames_dir, sprintf('frame_%04d.png', ti));
        imwrite(im, pngfn);
    end
    pause(0.02);
end

fprintf('Animation saved to %s\n', giffile);

function [nodes, prev_pts] = solve_leg_pose(node2_pos, node4, link_lengths, prev_pts, flip_logic)
    % 求解给定 node2 坐标和 node4 的单条腿姿态，返回当前节点与更新后的 prev_pts
    nodes = NaN(8,2);
    nodes(4,:) = node4;
    nodes(1,:) = [0, 0];
    if isempty(prev_pts) || any(size(prev_pts) ~= [8,2])
        prev_pts = NaN(8,2);
    end

    % 2: 使用共享的驱动点
    nodes(2,:) = node2_pos;

    % 3: d(3,2)=link2 and d(3,4)=link3
    pts = circle_intersections(nodes(2,:), link_lengths(2), nodes(4,:), link_lengths(3));
    if isempty(pts), return; end
    nodes(3,:) = choose_point(pts, prev_pts(3,:), 3, flip_logic);

    % 5: d(5,3)=link4 and d(5,4)=link5
    pts = circle_intersections(nodes(3,:), link_lengths(4), nodes(4,:), link_lengths(5));
    if isempty(pts), return; end
    nodes(5,:) = choose_point(pts, prev_pts(5,:), 5, flip_logic);

    % 6: d(6,2)=link7 and d(6,4)=link6
    pts = circle_intersections(nodes(2,:), link_lengths(7), nodes(4,:), link_lengths(6));
    if isempty(pts), return; end
    nodes(6,:) = choose_point(pts, prev_pts(6,:), 6, flip_logic);

    % 7: d(7,6)=link8 and d(7,5)=link9
    pts = circle_intersections(nodes(6,:), link_lengths(8), nodes(5,:), link_lengths(9));
    if isempty(pts), return; end
    nodes(7,:) = choose_point(pts, prev_pts(7,:), 7, flip_logic);

    % 8: d(8,6)=link10 and d(8,7)=link11
    pts = circle_intersections(nodes(6,:), link_lengths(10), nodes(7,:), link_lengths(11));
    if isempty(pts), return; end
    nodes(8,:) = choose_point(pts, prev_pts(8,:), 8, flip_logic);

    prev_pts = nodes;
end

function pts = circle_intersections(c1,r1,c2,r2)
    d = norm(c2-c1);
    if d < 1e-12
        pts = [];
        return;
    end
    if d > r1 + r2 + 1e-9 || d < abs(r1 - r2) - 1e-9
        pts = [];
        return;
    end
    a = (r1^2 - r2^2 + d^2) / (2*d);
    h = sqrt(max(0, r1^2 - a^2));
    p = c1 + a*(c2 - c1)/d;
    rx = - (c2(2)-c1(2)) * (h/d);
    ry =   (c2(1)-c1(1)) * (h/d);
    p1 = p + [rx, ry];
    p2 = p - [rx, ry];
    pts = [p1; p2];
end

function sel = choose_point(pts, prev, idx, flip_logic)
    if isempty(pts)
        sel = [NaN, NaN];
        return;
    end
    if any(isnan(prev))
        % 首次选择：仅在左右选择上取反；上下逻辑保持一致
        switch idx
            case 3 % node3: larger_y
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            case 5 % node5: 左腿取更小 x，右腿取更大 x
                if flip_logic
                    if pts(1,1) >= pts(2,1), sel = pts(1,:); else sel = pts(2,:); end
                else
                    if pts(1,1) <= pts(2,1), sel = pts(1,:); else sel = pts(2,:); end
                end
            case {6,7,8} % smaller_y，左右一致
                if pts(1,2) <= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            otherwise % 默认 larger_y，左右一致
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
        end
        return;
    end
    % 有上一帧时：左右都取更近的解（不反转上下）
    d1 = norm(pts(1,:) - prev);
    d2 = norm(pts(2,:) - prev);
    if d1 <= d2
        sel = pts(1,:);
    else
        sel = pts(2,:);
    end
end