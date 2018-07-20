function data = process_SensorMsgs_Joy(data_raw, time)

%time r p y thrust ??
data = zeros(length(data_raw), 5);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2) = data_raw{i}.Axes(1);
    data(i,3) = data_raw{i}.Axes(2);
    data(i,4) = data_raw{i}.Axes(3);
    data(i,5) = data_raw{i}.Axes(4);
end

end