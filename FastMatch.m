%根据当前位姿的栅格地图 优化预测的下一位姿 使下一位姿的栅格地图与当前位姿的栅格地图达到最大的重合度
%快速扫描匹配(请注意这可能会陷入局部最小值)
function [pose, bestHits] = FastMatch(gridmap, scan, pose, searchResolution)
%--------------------------------------------------------------------------
%输入
%   gridmap为局部栅格地图
%   scan为构成gridmap的当前扫描点集的局部笛卡尔坐标
%   pose为预测的下一位姿(预测得到的pose_guess)
%   searchResolution为搜索的分辨率(为主函数中预设的扫描匹配参数 [0.05; 0.05; deg2rad(0.5)] ) 
%输出
%   pose为优化过后的 预测下一位姿 优化目标函数是使下一位姿的栅格地图与当前位姿的栅格地图达到最大的重合度
%   bestHits 为pose对应的 最佳重合度score对应的 当前位姿栅格地图的原始距离矩阵
%--------------------------------------------------------------------------

%局部栅格地图信息
% Grid map information
metricMap = gridmap.metricMap;%栅格地图中0元素所在的位置靠近非零元素位置的最短栅格距离构成的矩阵
ipixel = 1 / gridmap.pixelSize;%实际距离1m对应几个栅格单元边长 (栅格单元尺寸对应的实际距离的倒数)
minX   = gridmap.topLeftCorner(1);%栅格地图中的最左端的横坐标(全局)
minY   = gridmap.topLeftCorner(2);%栅格地图中的最下端的纵坐标(全局)
nCols  = size(metricMap, 2);
nRows  = size(metricMap, 1);

%贪心算法
% Go down the hill
maxIter = 50;%最大循环次数
maxDepth = 3;%提高分辨率的次数的最大值
iter = 0;%循环变量
depth = 0;%分辨率提高次数

pixelScan = scan * ipixel;%将 扫描数据 实际坐标 转化为 栅格地图中的栅格坐标
bestPose  = pose;
bestScore = Inf;
t = searchResolution(1);%x和y坐标的搜索分辨率
r = searchResolution(3);%theta的搜索分辨率

while iter < maxIter
    noChange = true;
    % 旋转
    % Rotation
    for theta = pose(3) + [-r, 0, r]%遍历这个三个旋转角 [旋转角-r 旋转角 旋转角+r]

        ct = cos(theta);
        st = sin(theta);
        S  = pixelScan * [ct, st; -st, ct];%旋转矫正
        % 转换
        % Translation
        for tx = pose(1) + [-t, 0, t]%遍历这三个横坐标 [预测位姿横坐标-t 预测位姿横坐标 预测位姿横坐标+t]
            Sx = round(S(:,1)+(tx-minX)*ipixel) + 1;%以minX minY为坐标原点
            for ty = pose(2) + [-t, 0, t]
                Sy = round(S(:,2)+(ty-minY)*ipixel) + 1;%以minX minY为坐标原点

                isIn = Sx>1 & Sy>1 & Sx<nCols & Sy<nRows;%筛选在范围内的点
                ix = Sx(isIn);%提取出下一位姿扫描栅格 落在当前栅格地图区域的部分 的横坐标(单位:栅格)
                iy = Sy(isIn);%提取出下一位姿扫描栅格 落在当前栅格地图区域的部分 的纵坐标(单位:栅格)

                % metric socre
                idx = iy + (ix-1)*nRows;%把下一位姿扫描栅格的二维坐标转换为一维坐标idx
                %metricMap为当前位姿栅格地图中 非占用点距离占用点的距离矩阵
                %score理解为 下一位姿扫描栅格与当前位姿扫描栅格的重合度(score约小 表示重合度越高)
                hits = metricMap(idx);%度量矩阵对应位置的值
                score = sum(hits);

                % update 
                if score < bestScore %目的是找到最低的score(即预测栅格与当前栅格达到最高重合度)
                    noChange  = false;
                    bestPose  = [tx; ty; theta];%将这个最高重合度的 预测位姿 作为最佳预测位姿
                    bestScore = score;
                    bestHits  = hits;
                end

            end
        end
    end
    % 找不到更好的匹配，提高分辨率
    if noChange
        r = r / 2;
        t = t / 2;
        depth = depth + 1;
        if depth > maxDepth %分辨率提高次数不能超过maxDepth
            break;
        end
    end
    pose = bestPose;%最佳位姿作为预测的下一位姿
    iter = iter + 1;
end