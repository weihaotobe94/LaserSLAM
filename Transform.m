%把局部坐标转化为全局坐标
function tscan = Transform(scan, pose)
%--------------------------------------------------------------------------
%输入 
%   pose为当前位姿(x坐标tx  y坐标ty  旋转角theta)
%   scan为某次扫描数据的局部笛卡尔坐标
%输出
%   tscan为 通过当前位姿pose 将当前扫描数据的局部笛卡尔坐标scan 转换为的全局笛卡尔坐标
%--------------------------------------------------------------------------
tx = pose(1);
ty = pose(2);
theta = pose(3);

ct = cos(theta);    
st = sin(theta);
R  = [ct, -st; st, ct];

tscan = scan * (R');
tscan(:,1) = tscan(:,1) + tx;
tscan(:,2) = tscan(:,2) + ty;