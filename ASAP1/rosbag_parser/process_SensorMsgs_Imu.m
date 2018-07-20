function data = process_SensorMsgs_Imu(data_raw, time)

%time r p y p q r ax ay az
data = zeros(length(data_raw), 10);
for i=1:length(data_raw)
    data(i,1) = time(i);
    quart = [data_raw{i}.Orientation.X data_raw{i}.Orientation.Y data_raw{i}.Orientation.Z data_raw{i}.Orientation.W].';
    rpy = quart2rpy(quart);
    data(i,2:4) = rpy.';
    data(i,5) = data_raw{i}.AngularVelocity.X;
    data(i,6) = data_raw{i}.AngularVelocity.Y;
    data(i,7) = data_raw{i}.AngularVelocity.Z;
    data(i,8) = data_raw{i}.LinearAcceleration.X;
    data(i,9) = data_raw{i}.LinearAcceleration.Y;
    data(i,10) = data_raw{i}.LinearAcceleration.Z;
end

end