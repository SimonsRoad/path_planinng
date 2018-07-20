function data = process_GeometryMsgs_PoseStamped(data_raw, time)

%time x y z roll pitch yaw
data = zeros(length(data_raw), 7);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2) = data_raw{i}.Pose.Position.X;
    data(i,3) = data_raw{i}.Pose.Position.Y;
    data(i,4) = data_raw{i}.Pose.Position.Z;
    quart = [data_raw{i}.Pose.Orientation.X data_raw{i}.Pose.Orientation.Y data_raw{i}.Pose.Orientation.Z data_raw{i}.Pose.Orientation.W].';
    rpy = quart2rpy(quart);
    data(i,5:7) = rpy.';
end

end