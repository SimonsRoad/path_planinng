function data = process_MavrosMsgs_AttitudeTarget(data_raw, time)

%time r p y p q r thrust
data = zeros(length(data_raw), 8);
for i=1:length(data_raw)
    data(i,1) = time(i);
    quart = [data_raw{i}.Orientation.X data_raw{i}.Orientation.Y data_raw{i}.Orientation.Z data_raw{i}.Orientation.W].';
    rpy = quart2rpy(quart);
    data(i,2:4) = rpy.';
    data(i,5) = data_raw{i}.BodyRate.X;
    data(i,6) = data_raw{i}.BodyRate.Y;
    data(i,7) = data_raw{i}.BodyRate.Z;
    data(i,8) = data_raw{i}.Thrust;
end

end