function data = process_StdMsgs_Float64MultiArray(data_raw, time)

%time data
data = zeros(length(data_raw), 2);
dataLength = length(data_raw{1}.Data);
for i=1:length(data_raw)
    data(i,1) = time(i);
    data(i,2:2+dataLength-1) = data_raw{i}.Data;
end

end