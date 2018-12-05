% 从全局地图中 提取当前扫描周围的局部地图 的全局坐标
% Extract a local map around current scan
function localMap = ExtractLocalMap(points, pose, scan, borderSize)
%--------------------------------------------------------------------------
%输入
%   points为全局地图点集
%   pose为当前位姿
%   scan为当前扫描数据的局部坐标
%   borderSize为
%--------------------------------------------------------------------------

% 将当前扫描数据坐标scan 转化为全局坐标scan_w
% Transform current scan into world frame
scan_w = Transform(scan, pose);
% 设置 左上角 和 右下角
% Set top-left & bottom-right corner
minX = min(scan_w(:,1) - borderSize);
minY = min(scan_w(:,2) - borderSize);
maxX = max(scan_w(:,1) + borderSize);
maxY = max(scan_w(:,2) + borderSize);
% 提取位于范围内的全局地图中的点
% Extract
isAround = points(:,1) > minX...
         & points(:,1) < maxX...
         & points(:,2) > minY...
         & points(:,2) < maxY;
%从全局地图中提取到的当前扫描点
localMap = points(isAround, :);