%求位姿差(旋转始终为0)
function dp = DiffPose(pose1, pose2)

    dp = pose2 - pose1;
    %dp(3) = angdiff(pose1(3), pose2(3));
    %dp(3) = pose2(3)-pose2(3);
end