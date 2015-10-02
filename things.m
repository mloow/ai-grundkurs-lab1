

pathFile = fopen('path.txt', 'r');
path = csvread('path.txt');
%fclose(pathFile);

cpsFile = fopen('cps.txt', 'r');
cps = csvread('cps.txt');
%fclose(cpsFile);

robotPos = csvread('robotPos.txt');

%plot(path(:, 1), path(:, 2), 'b', robotPos(:, 1), robotPos(:, 2), 'g', cps(:, 1), cps(:,2), 'ro'); 
plot(path(:, 1), path(:, 2), 'b', robotPos(:, 1), robotPos(:, 2), 'g'); 
%plot(path(:, 1), path(:, 2));