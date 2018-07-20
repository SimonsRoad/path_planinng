%% MATLAB ROS bag file reader
%Clark Youngdong Son
%E-mail : clark.y.d.son@gmail.com
%Currently only general message types are supported

%History
%2017-11-29 : Initial flow is generated to draw some topics from a sample

%Bug
%If the chosen topics are not supported -> require action
clear; close all; clc;
%% Supporting message types
supportTypes = {'geometry_msgs/PoseStamped';
                'geometry_msgs/TwistStamped'
                'geometry_msgs/PoseArray';
                'sensor_msgs/Imu';
                'sensor_msgs/Joy';
                'sensor_msgs/Image';
                'visualization_msgs/MarkerArray';
                'mavros_msgs/PositionTarget';  %mavros_msgs messages must be built in your MATLAB
                'mavros_msgs/AttitudeTarget';
                'std_msgs/String';
                'std_msgs/Float64';
                'std_msgs/Float64MultiArray';
                'nav_msgs/Odometry'};
            
%% Load rosbag file
fprintf('\nLoad one of the following ROS bag files\n');
files = dir;
L = length(files);
index = zeros(2, L);
count = 1;
for i = 1:L
    M = length(files(i).name);
    if M > 4 && strcmp(files(i).name(M-3:M), '.bag')
        index(i) = true;
        fprintf('%-10d %-35s\n', count, files(i).name);
        index(2,count) = i;
        count = count + 1;
    end
end
fileIndex = input('\nChoose one file : ');

if (count==1)
    disp('No rosbag found in the current folder');
    return;
elseif(index(2,fileIndex)==0)
    fprintf('Not in the desired range. Desired range : %3d ~ %-3d\n', 1, count-1);
    return;
end

fileName = files(index(2,fileIndex)).name;
bag = rosbag(fileName);
%% Choose which topics you want to log
fprintf('\nTopic list and support\n');
topicNumber = size(bag.AvailableTopics,1);
for i=1:topicNumber
    %disp([num2str(i) '         ' bag.AvailableTopics.Properties.RowNames{i}]);
    topicName = bag.AvailableTopics.Properties.RowNames{i};
    topicType = cellstr(bag.AvailableTopics(i,2).Variables);
    topicType = topicType{1};
    result = false;
    for j=1:length(supportTypes)
        if(strcmp(topicType, supportTypes{j}))
            result = true;
            break;
        end
    end
    if(result)
       fprintf('%-10d topic: %-35s     type: %-33s    supported\n', i, topicName, topicType);
    else
       fprintf(2,'%-10d topic: %-35s     type: %-33s    not supported\n', i, topicName, topicType);  
    end
end
prompt = 'Choose topics to display (ex: 1 3 5 7. Type enter to log all topics) : ';

interestedTopic = input(prompt, 's');
if(isempty(interestedTopic))
    interestedTopic = 1:topicNumber;
else
    interestedTopic = str2num(interestedTopic);
    if(isempty(interestedTopic))
        disp('topic numbers are invalid. insert a space between numbers');
        topicNumber = 0;
    else
        topicNumber = length(interestedTopic);
    end
end

%% Check supported message types
fprintf('\nMessage types support check\n');
supportFlag = zeros(1,topicNumber);
for i=1:topicNumber
    topicName = bag.AvailableTopics.Row{interestedTopic(i)};
    topicType = cellstr(bag.AvailableTopics(interestedTopic(i),2).Variables);
    topicType = topicType{1};
    result = false;
    for j=1:length(supportTypes)
        if(strcmp(topicType, supportTypes{j}))
            supportFlag(i) = true;
            result = true;
            break;
        end
    end
    if(result)
       fprintf('%-10d topic: %-35s     type: %-33s    supported\n', interestedTopic(i), topicName, topicType);
    else
       fprintf(2,'%-10d topic: %-35s     type: %-33s    not supported\n', interestedTopic(i), topicName, topicType);  
    end
end

%% Log data
fprintf('\nProcessing\n');
supportNumber = sum(supportFlag);
data = cell(supportNumber,2);
count = 0;
for i=1:topicNumber
    if(supportFlag(i))
        topicName = bag.AvailableTopics.Row{interestedTopic(i)};
        fprintf('%-10d topic: %-35s   %2.0f%% done\n', interestedTopic(i), topicName, 100*count/sum(supportFlag));
        drawnow;
        bagTemp = select(bag, 'Topic', topicName);
        data_raw = readMessages(bagTemp);
        data{i,1} = topicName;
        data{i,2} = processData(data_raw, bagTemp.MessageList{:,1});
        count = count + 1;
    end
end
fprintf('%-52s   %2.0f%% done\n', [], 100);
fprintf('\nSaving the data\n');
fprintf('\nDone\n');
save([fileName(1:end-4) '.mat'], 'data');