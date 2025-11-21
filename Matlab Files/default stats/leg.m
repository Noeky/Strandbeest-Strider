theta = 0;
link_lengths = [15; 50; 41.5; 55.8; 40.1; 39.3; 61.9; 36.7; 39.4; 49; 65.7];

nodes = NaN(8,2);
nodes(4,:) = [-38, -7.8];
fixedMask = false(8,1);
fixedMask(4) = true;
nodes(1,:) = [0, 0];
fixedMask(1) = true;
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

% 解析求解各点
pivot_idx = 1;
driven_idx = 2;
crank_len = link_lengths(1);

theta_vec = linspace(0,2*pi,361);
end_traj = zeros(length(theta_vec),2);

% 存储前一步解的变量
prev_pts = NaN(8,2);

% 保存每一步的节点位置，用于动画
nodes_history = zeros(length(theta_vec), 8, 2);

for ti = 1:length(theta_vec)
    theta = theta_vec(ti);

    % 2: polar from pivot
    pivot_pos = nodes(pivot_idx,:);
    nodes(2,:) = pivot_pos + crank_len*[cos(theta), sin(theta)];

    % 3: d(3,2)=link2 and d(3,4)=link3
    pts = circle_intersections(nodes(2,:), link_lengths(2), nodes(4,:), link_lengths(3));
    nodes(3,:) = choose_point(pts, prev_pts(3,:), 3);

    % 5: d(5,3)=link4 and d(5,4)=link5
    pts = circle_intersections(nodes(3,:), link_lengths(4), nodes(4,:), link_lengths(5));
    nodes(5,:) = choose_point(pts, prev_pts(5,:), 5);

    % 6: d(6,2)=link7 and d(6,4)=link6
    pts = circle_intersections(nodes(2,:), link_lengths(7), nodes(4,:), link_lengths(6));
    nodes(6,:) = choose_point(pts, prev_pts(6,:), 6);

    % 7: d(7,6)=link8 and d(7,5)=link9
    pts = circle_intersections(nodes(6,:), link_lengths(8), nodes(5,:), link_lengths(9));
    nodes(7,:) = choose_point(pts, prev_pts(7,:), 7);

    % 8: d(8,6)=link10 and d(8,7)=link11
    pts = circle_intersections(nodes(6,:), link_lengths(10), nodes(7,:), link_lengths(11));
    nodes(8,:) = choose_point(pts, prev_pts(8,:), 8);

    prev_pts = nodes; % 更新上一步估计

    end_traj(ti,:) = nodes(8,:);
    nodes_history(ti,:,:) = nodes;
end

figure('Name','End point 8 trajectory','NumberTitle','off'); hold on; axis equal; grid on;
plot(end_traj(:,1), end_traj(:,2), '-b', 'LineWidth', 1.5);
plot(nodes(4,1), nodes(4,2), 'ro');
title('Trajectory of node 8'); xlabel('X'); ylabel('Y');

% 生成并保存机构动图，以及在窗口中播放
giffile = fullfile(pwd,'leg_animation.gif');
fig = figure('Name','Leg Animation','NumberTitle','off'); axis equal; grid on; hold on;
pad = 10; % 视图边界扩展

% 计算绘图范围
xlim([xmin xmax]); ylim([ymin ymax]);
all_coords = reshape(nodes_history, [], 2);
valid = all(~isnan(all_coords),2);
all_coords = all_coords(valid,:);
if isempty(all_coords)
    error('No valid node coordinates were computed; check fixed nodes and link lengths.');
end
xmin = min(all_coords(:,1)) - pad; xmax = max(all_coords(:,1)) + pad;
ymin = min(all_coords(:,2)) - pad; ymax = max(all_coords(:,2)) + pad;
xlim([xmin xmax]); ylim([ymin ymax]);
xlim([xmin xmax]); ylim([ymin ymax]);

for ti = 1:size(nodes_history,1)
    clf; hold on; axis equal; grid on;
    curr = squeeze(nodes_history(ti,:,:));
    for k = 1:size(links_pairs,1)
        i = links_pairs(k,1); j = links_pairs(k,2);
        Xi = curr(i,:); Xj = curr(j,:);
        plot([Xi(1) Xj(1)], [Xi(2) Xj(2)], '-k', 'LineWidth', 2);
        plot(Xi(1), Xi(2), 'ko', 'MarkerFaceColor','w');
        plot(Xj(1), Xj(2), 'ko', 'MarkerFaceColor','w');
    end
    title(sprintf('Theta = %.1f°', rad2deg(theta_vec(ti))));
    xlim([xmin xmax]); ylim([ymin ymax]);
    drawnow;

    % 写入 GIF
    frame = getframe(fig);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if ti == 1
        imwrite(imind,cm,giffile,'gif','Loopcount',inf,'DelayTime',0.03);
    else
        imwrite(imind,cm,giffile,'gif','WriteMode','append','DelayTime',0.03);
    end
    pause(0.02);
end

fprintf('Animation saved to %s\n', giffile);

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

function sel = choose_point(pts, prev, idx)
    if isempty(pts)
        error('No intersection points found for given radii/centers.');
    end
    if any(isnan(prev))
        % 首次选择：按节点指定的规则选择
        switch idx
            case 3 % node3: larger_y
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            case 5 % node5: smaller_x
                if pts(1,1) <= pts(2,1), sel = pts(1,:); else sel = pts(2,:); end
            case {6,7,8} % smaller_y
                if pts(1,2) <= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
            otherwise % 默认 larger_y
                if pts(1,2) >= pts(2,2), sel = pts(1,:); else sel = pts(2,:); end
        end
        return;
    end
    d1 = norm(pts(1,:) - prev);
    d2 = norm(pts(2,:) - prev);
    if d1 <= d2
        sel = pts(1,:);
    else
        sel = pts(2,:);
    end
end

