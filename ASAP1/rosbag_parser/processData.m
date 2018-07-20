function data = processData(data_raw, time)

topicType = data_raw{1}.MessageType;
%standard
if(strcmp(topicType, 'geometry_msgs/PoseStamped'))
    data = process_GeometryMsgs_PoseStamped(data_raw, time);
elseif(strcmp(topicType, 'geometry_msgs/TwistStamped'))
    data = process_GeometryMsgs_TwistStamped(data_raw, time);
elseif(strcmp(topicType, 'geometry_msgs/PoseArray'))
    data = process_GeometryMsgs_PoseArray(data_raw, time);
elseif(strcmp(topicType, 'visualization_msgs/MarkerArray'))
    data = process_VisualizationMsgs_MarkerArray(data_raw, time);
elseif(strcmp(topicType, 'sensor_msgs/Imu'))
    data = process_SensorMsgs_Imu(data_raw, time);
elseif(strcmp(topicType, 'sensor_msgs/Joy'))
    data = process_SensorMsgs_Joy(data_raw, time);
elseif(strcmp(topicType, 'sensor_msgs/Image'))
    data = process_SensorMsgs_Image(data_raw, time);
elseif(strcmp(topicType, 'std_msgs/String'))
    data = process_StdMsgs_String(data_raw, time);
elseif(strcmp(topicType, 'std_msgs/Float64'))
    data = process_StdMsgs_Float64(data_raw, time);
elseif(strcmp(topicType, 'std_msgs/Float64MultiArray'))
    data = process_StdMsgs_Float64MultiArray(data_raw, time);
elseif(strcmp(topicType, 'nav_msgs/Odometry'))
    data = process_NavMsgs_Odometry(data_raw, time);
%mavros
elseif(strcmp(topicType, 'mavros_msgs/PositionTarget'))
    data = process_MavrosMsgs_PositionTarget(data_raw, time);
elseif(strcmp(topicType, 'mavros_msgs/AttitudeTarget'))
    data = process_MavrosMsgs_AttitudeTarget(data_raw, time);
end

end