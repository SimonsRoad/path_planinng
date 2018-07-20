function data = process_GeometryMsgs_PoseArray(data_raw, time)

%time poseNumber x1 y1 z1 roll1 pitch1 yaw1 x2 y2 z2 roll2 pitch2 yaw2 ...
maxPoseNumber = 0;
poseNumberIdx = [];
for i=1:length(data_raw)
    poseNumber = length(data_raw{i}.Poses);
    if(poseNumber>0)
        poseNumberIdx = [poseNumberIdx; i];
    end
    if(poseNumber>maxPoseNumber)
        maxPoseNumber = poseNumber;
    end
end

data = zeros(length(poseNumberIdx), 2+3*maxPoseNumber);
for ii=1:length(poseNumberIdx)
    i = poseNumberIdx(ii);
    data(ii,1) = time(i);
    poseNumber = length(data_raw{i}.Poses);
    data(ii,2) = poseNumber;
    for j=1:poseNumber
        data(ii,2+6*(j-1)+1) = data_raw{i}.Poses(j).Position.X;
        data(ii,2+6*(j-1)+2) = data_raw{i}.Poses(j).Position.Y;
        data(ii,2+6*(j-1)+3) = data_raw{i}.Poses(j).Position.Z;
        quart = [data_raw{i}.Poses(j).Orientation.X data_raw{i}.Poses(j).Orientation.Y data_raw{i}.Poses(j).Orientation.Z data_raw{i}.Poses(j).Orientation.W].';
        rpy = quart2rpy(quart);
        data(ii,2+6*(j-1)+4:2+6*(j-1)+6) = rpy.';
    end
end

end