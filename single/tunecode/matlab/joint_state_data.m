function  [pos, vel, eff, sec, nsec, names] = joint_state_data(bagfilename, topicname)
%
%   [pos, vel, eff, sec, nsec, names] = joint_state_data(bagfilename, topicname)
%
%   Extract the positions, velocities, and efforts from joint_state
%   messages as well as the sample times (in sec/nsec) and joint
%   names.  Use the topic 'topicname' in the bag file 'bagfilename'.
%
%   Each sample time is a row, each joint is in a column.
%

% Clear the output arguments (in case of problems).
[pos, vel, eff, sec, nsec] = deal([]);
names = {};

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end

% Isolate the specified topic.
msgs = select(bag, 'Topic', topicname);
if (~msgs.NumMessages)
    warning(['No messages under topic ''' topicname '''']);
    return;
end

% Convert into structure array.
data = cell2mat(readMessages(msgs, 'DataFormat', 'struct'));

% Double-check the type.
if (~strcmp(data(1).MessageType, 'sensor_msgs/JointState'))
    warning(['Topic ''' topicname ''' is not of type sensor_msgs/JointState']);
    return;
end

% Extract the names.
names = data(1).Name';

% Extract the data.
pos  = double([data(:).Position]');
vel  = double([data(:).Velocity]');
eff  = double([data(:).Effort]');

headers = [data(:).Header];
stamps  = [headers(:).Stamp];

sec  = double([stamps(:).Sec]');
nsec = double([stamps(:).Nsec]');
