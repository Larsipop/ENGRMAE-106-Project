clear;
clc;

data = load('Coolterm Capture - Arduino 106 Unit Test 2.txt');

robotCoords = [data(:, 1:2)];
robotAim = [data(:, 3:4)];
goal = [data(1, 5:6)];

hold on
grid on
for i = 1:size(data, 1)
    plot([robotCoords(i,1), robotAim(i,1)], [robotCoords(i,2), robotAim(i,2)], '-s', 'MarkerIndices', 1); 
end

plot(goal, '*')

xlim([-1500, 1500])
ylim([-1500, 1500])
xticks(-1500:250:1500);
yticks(-1500:250:1500);

title('Unit Tester: Robot Aim with Respect to Location (X, Y)')
xlabel('Position (X)')
ylabel('Position (Y)')
%annotation('textbox', 'SouthWest', 'String', 'O marker appears at the robot position, and the line points in the direction of the robots aim vector', 'FitBoxToText', 'on')
%TextH = text();
hold off