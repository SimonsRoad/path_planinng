function data = process_SensorMsgs_Image(data_raw, time)

%time width height image_data
data = cell(length(data_raw), 1);
for i=1:length(data_raw)
    data{i,1} = time(i);
    data{i,2} = readImage(data_raw{i});
end

end