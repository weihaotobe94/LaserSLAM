%将LiDARd第idx次扫描数据从极坐标转化为笛卡尔坐标(相对于小车的局部坐标)
% Read a laser scan
function scan = ReadAScan(lidar_data, idx, lidar, usableRange)
%--------------------------------------------------------------------------
% 输入:
%lidar_data为读取的LiDAR扫描数据
%idx为扫描次数的索引值
%lidar为由SetLidarParameters()设置的LiDAR参数
%usableRange为可使用的范围
%--------------------------------------------------------------------------
    angles = lidar.angles;%
    ranges = lidar_data.ranges(idx, :)'; %选取LiDAR数据的ranges中idx索引对应的这次扫描的数据
    % 删除范围不太可靠的点
    % Remove points whose range is not so trustworthy
    maxRange = min(lidar.range_max, usableRange);
    isBad = ranges < lidar.range_min | ranges > maxRange;%ranges中小于最小范围或大于最大范围d的 数据的 索引下标
    angles(isBad) = [];
    ranges(isBad) = [];
    % 从极坐标转换为笛卡尔坐标
    % Convert from polar coordinates to cartesian coordinates
    [xs, ys] = pol2cart(angles, ranges);%(angles, ranges)为极坐标中的(theta,rho)
    scan = [xs, ys];  
end