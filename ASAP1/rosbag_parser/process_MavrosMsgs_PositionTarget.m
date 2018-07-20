function data = process_MavrosMsgs_PositionTarget(data_raw, time)

%time x y z vx vy vz yaw yawRate
data = zeros(length(data_raw), 9);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2) = data_raw{i}.Position.X;
    data(i,3) = data_raw{i}.Position.Y;
    data(i,4) = data_raw{i}.Position.Z;
    data(i,5) = data_raw{i}.Velocity.X;
    data(i,6) = data_raw{i}.Velocity.Y;
    data(i,7) = data_raw{i}.Velocity.Z;
    data(i,8) = data_raw{i}.Yaw;
    data(i,9) = data_raw{i}.YawRate;
end

end