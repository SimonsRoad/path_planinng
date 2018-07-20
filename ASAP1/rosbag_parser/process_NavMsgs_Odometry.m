function data = process_NavMsgs_Odometry(data_raw, time)

%time x y z r p y vx vy vz omega_x omega_y omega_z
data = zeros(length(data_raw), 13);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2) = data_raw{i}.Pose.Pose.Position.X;
    data(i,3) = data_raw{i}.Pose.Pose.Position.Y;
    data(i,4) = data_raw{i}.Pose.Pose.Position.Z;
    quart = [data_raw{i}.Pose.Pose.Orientation.X;
        data_raw{i}.Pose.Pose.Orientation.Y;
        data_raw{i}.Pose.Pose.Orientation.Z;
        data_raw{i}.Pose.Pose.Orientation.W].';
    rpy = quart2rpy(quart);
    data(i,5:7) = rpy.';
    data(i,8) = data_raw{i}.Twist.Twist.Linear.X;
    data(i,9) = data_raw{i}.Twist.Twist.Linear.Y;
    data(i,10) = data_raw{i}.Twist.Twist.Linear.Z;
    data(i,11) = data_raw{i}.Twist.Twist.Angular.X;
    data(i,12) = data_raw{i}.Twist.Twist.Angular.Y;
    data(i,13) = data_raw{i}.Twist.Twist.Angular.Z;
end

end