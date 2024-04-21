% Example datasets of 3D coordinates (replace with your actual data)


% Initialize figure and scatter plot
figure;
scatterHandleY = scatter3(NaN, NaN, NaN, 'filled', 'MarkerFaceColor', 'b', 'SizeData', 5); % Empty scatter plot for y (blue)
hold on;
scatterHandleTargets = scatter3(NaN, NaN, NaN, 'filled', 'MarkerFaceColor', 'r', 'SizeData', 5); % Empty scatter plot for targets (red)
hold off;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Animation of Points Appearing');

% Animation loop
for i = 1:size(targets, 1)
    % Add one point to the scatter plot for y
    set(scatterHandleY, 'XData', y(1:i, 1), ...
                        'YData', y(1:i, 2), ...
                        'ZData', y(1:i, 3));
    
    % Add one point to the scatter plot for targets
    set(scatterHandleTargets, 'XData', targets(1:i, 1), ...
                              'YData', targets(1:i, 2), ...
                              'ZData', targets(1:i, 3));
    
    % Pause for a short duration (adjust as needed)
    pause(0.1); % 0.1 seconds pause
    
    % Update plot
    drawnow;
end
