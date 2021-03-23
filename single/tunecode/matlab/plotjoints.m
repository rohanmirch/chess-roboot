function plotjoints(bagfilename, joint)
%
%   plotjoints(bagfilename, joint)
%
%   Plot the command and feedback joint_state topics saved in the
%   bagfile.  If 'bagfilename' is not given or given as 'latest', use
%   the most recent bag file.  If 'joint' is given, plot only the
%   joint given by an index (number) or name (string).
%
    
% If no bagfile is specified, use the most recent.
if (~exist('bagfilename') || strcmp(bagfilename, 'latest'))
    % Get a list of .bag files in the current folder.
    d = dir('*.bag');

    % Make sure we have at least one bag file.
    if (~size(d,1))
        error('Unable to find a bag file');
    end

    % Find the most recently modified file
    [~, idx] = max([d.datenum]);

    % Use as the bag file.
    bagfilename = d(idx).name;
    disp(['Using bag file ''' bagfilename '''']);
end

% Pick the topics.
topic_desired = '/hebiros/robot/command/joint_state';
topic_actual  = '/hebiros/robot/feedback/joint_state';

% Extract the data.
[pd, vd, ed, secd, nsecd, nd] = joint_state_data(bagfilename, topic_desired);
[pa, va, ea, seca, nseca, na] = joint_state_data(bagfilename, topic_actual);

% Find the earlier start time and use as time 0.
if (isempty(secd) && isempty(seca))
    error(['No joint data in bag file ''' bagfilename '''']);
elseif (isempty(secd))
    td = [];
    ta = (seca - seca(1)) + 1e-9 * (nseca - nseca(1));
elseif (isempty(seca))
    td = (secd - secd(1)) + 1e-9 * (nsecd - nsecd(1));
    ta = [];
else
    sec0 = min([secd(1) seca(1)]);
    td   = (secd - sec0) + 1e-9 * nsecd;
    ta   = (seca - sec0) + 1e-9 * nseca;
    t0   = min([td(1) ta(1)]);
    td   = (td - t0);
    ta   = (ta - t0);
end

% Re-sort the actual joints (columns of actual data) to match the
% order of the desired joints - in case they aren't ordered the same!
if (isequal(sort(nd), sort(na)))
    [~, id] = sort(nd);
    [~, ia] = sort(na);

    if (~isequal(id, ia))
        warning('Reording actual joints to match desired order');
        pa(:, id) = pa(:, ia);
        va(:, id) = va(:, ia);
        ea(:, id) = ea(:, ia);
        na(id)    = na(ia);
    end
end

% Potentially plot only a single joint.
if (exist('joint') && (~strcmp(joint, 'all')))
    if (isnumeric(joint) && (numel(joint) == 1))
        if ((length(nd) > 0) && (joint >= 1) && (joint <= length(nd)))
            joint = nd{floor(joint)};
        elseif ((length(na) > 0) && (joint >= 1) && (joint <= length(na)))
            joint = na{floor(joint)};
        else
            error(['Out of range joint index ' num2str(joint)]);
        end
    elseif (~ischar(joint))
        error('Bad joint index argument');
    end
    ia = find(strcmp(na, joint));
    id = find(strcmp(nd, joint));
    if (isempty(ia) && isempty(id))
        error(['Unable to find joint ''' joint ''''])
    end
    disp(['Plotting only joint ''' joint '''']);
    pa = pa(:, ia);
    va = va(:, ia);
    ea = ea(:, ia);
    na = na(ia);
    pd = pd(:, id);
    vd = vd(:, id);
    ed = ed(:, id);
    nd = nd(id);
end


% Plot.  Note we (1) graph the actual data first so we can show the
% legend for the actual data only.  And (2) we reset the color list,
% so the desired matches the actual colors.
figure(gcf);
clf;

ax(1) = subplot(3,1,1);
grid on;
hold on;
plot(ta, pa, '-');
set(gca, 'ColorOrderIndex', 1);
plot(td, pd, '--');
ylabel('Position (rad)');

if (isequal(na, nd))
    legend(na);
else
    legend({na{:}, nd{:}});
end
title([bagfilename '  ---  Command (dashed) and Feedback (solid)']);

ax(2) = subplot(3,1,2);
grid on;
hold on;
plot(ta, va, '-');
set(gca, 'ColorOrderIndex', 1);
plot(td, vd, '--');
ylabel('Velocity (rad/sec)');

ax(3) = subplot(3,1,3);
grid on;
hold on;
plot(ta, ea, '-');
set(gca, 'ColorOrderIndex', 1);
plot(td, ed, '--');
ylabel('Effort (Nm)');

xlabel('Time (sec)');
grid on;
linkaxes(ax, 'x');

set(gcf, 'Name', 'Joint Data');
set(gcf, 'PaperPosition', [0.2500    0.2500    8.0000   10.5000]);
