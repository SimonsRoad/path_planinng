function data = process_GeometryMsgs_TwistStamped(data_raw, time)

%time vx vy vz p q r
data = zeros(length(data_raw), 7);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2) = data_raw{i}.Twist.Linear.X;
    data(i,3) = data_raw{i}.Twist.Linear.Y;
    data(i,4) = data_raw{i}.Twist.Linear.Z;
    data(i,5) = data_raw{i}.Twist.Angular.X;
    data(i,6) = data_raw{i}.Twist.Angular.Y;
    data(i,7) = data_raw{i}.Twist.Angular.Z;
end

end