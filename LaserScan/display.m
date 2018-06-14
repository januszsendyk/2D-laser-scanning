data=importdata('cloud_0.txt');
data=data.*17 %mm
figure (1)
scatter3(data(1,2:end),data(2,2:end),data(3,2:end), 1);
figure (2)
plot(data(1,2:end),data(3,2:end),'.')

data=importdata('cloud_1.txt');
data=data.*17 %mm
figure (1)
hold on;
scatter3(data(1,2:end),data(2,2:end),data(3,2:end), 1);
hold off;
figure (2)
hold on;
plot(data(1,2:end),data(3,2:end),'r.')
hold off;