function data = process_VisualizationMsgs_MarkerArray(data_raw, time)

%time markerNumber x1 y1 z1 roll1 pitch1 yaw1 x2 y2 z2 roll2 pitch2 yaw2 ...
maxMarkerNumber = 0;
markerNumberIdx = [];
for i=1:length(data_raw)
    markerNumber = length(data_raw{i}.Markers);
    if(markerNumber>0)
        markerNumberIdx = [markerNumberIdx; i]; 
    end
    if(markerNumber>maxMarkerNumber)
        maxMarkerNumber = markerNumber;
    end
end

data = zeros(length(markerNumberIdx), 2+3*maxMarkerNumber);
for ii=1:length(markerNumberIdx)
    i = markerNumberIdx(ii);
    data(ii,1) = time(i);
    markerNumber = length(data_raw{i}.Markers);
    data(ii,2) = markerNumber;
    for j=1:markerNumber
        data(ii,2+6*(j-1)+1) = data_raw{i}.Markers(j).Pose.Position.X;
        data(ii,2+6*(j-1)+2) = data_raw{i}.Markers(j).Pose.Position.Y;
        data(ii,2+6*(j-1)+3) = data_raw{i}.Markers(j).Pose.Position.Z;
        quart = [data_raw{i}.Markers(j).Pose.Orientation.X data_raw{i}.Markers(j).Pose.Orientation.Y data_raw{i}.Markers(j).Pose.Orientation.Z data_raw{i}.Markers(j).Pose.Orientation.W].';
        rpy = quart2rpy(quart);
        data(ii,2+6*(j-1)+4:2+6*(j-1)+6) = rpy.';
    end
end

end