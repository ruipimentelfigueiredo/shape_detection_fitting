iterations=1200;
x_upper_limit=0.31;

%% orientation
orientation_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_clutter_0.txt';
%orientation_file_1='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_clutter_1.txt';
orientation_file_hybrid='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_clutter_1.txt';

formatSpec = '%f';
sizeA = [Inf iterations];
%A = fscanf(fileID,formatSpec,sizeA)
orientations = importdata(orientation_file);
orientations = (180.0/pi)*orientations;
orientations_mean = mean(orientations,2);
orientations_std_dev=std(orientations,0,2);

% orientations_1 = importdata(orientation_file_1);
% orientations_1 = (180.0/pi)*orientations_1;
% orientations_1_mean = mean(orientations_1,2);
% orientations_1_std_dev=std(orientations_1,0,2);

orientations_hybrid = importdata(orientation_file_hybrid);
orientations_hybrid = (180.0/pi)*orientations_hybrid;
orientations_hybrid_mean = mean(orientations_hybrid,2);
orientations_hybrid_std_dev=std(orientations_hybrid,0,2);


% PLOTS
close all

figure(1)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(orientations_mean)
hold on
errorbar(100*[0:0.5:2.0],orientations_mean,orientations_std_dev,'--b')
%errorbar([0:0.05:0.5],orientations_1_mean,orientations_1_std_dev,'k')
errorbar(100*[0:0.5:2.0],orientations_hybrid_mean,orientations_hybrid_std_dev,'k')


legend('Rabbani et al.','Ours')
xlabel('clutter (% of cylinder surface points)','Interpreter','LaTex','FontSize',fontsize);
ylabel('absolute orientation error (º)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',100*[0:0.5:2.0])

axis ([-1 200 0 95]) 
set(gca,'fontsize',fontsize);

scale = 0.1;
pos = get(gca, 'Position');
pos(2) = pos(2)+scale*pos(4);
pos(4) = (1-scale)*pos(4);
set(gca, 'Position', pos)

export_fig clutter_orientation_error -pdf

%% ORIENTATIONS REC
% orientations_rec=orientations;
% orientations_rec(orientations_rec<=12)=1
% orientations_rec(orientations_rec>12)=0
% ori_rec_rate=100.0*sum(orientations_rec,2)/iterations;
% 
% orientations_rec_1=orientations_1;
% orientations_rec_1(orientations_rec_1<=12)=1
% orientations_rec_1(orientations_rec_1>12)=0
% ori_rec_rate_1=100.0*sum(orientations_rec_1,2)/iterations
% 
% orientations_rec_hybrid=orientations_hybrid;
% orientations_rec_hybrid(orientations_rec_hybrid<=12)=1
% orientations_rec_hybrid(orientations_rec_hybrid>12)=0
% ori_rec_rate_hybrid=100.0*sum(orientations_rec_hybrid,2)/iterations;


% figure(2)
% fontsize=15;
% set(gcf, 'Color', [1,1,1]);
% hold on
% plot([0:0.05:0.5],ori_rec_rate,'--b')
% plot([0:0.05:0.5],ori_rec_rate_1,'k')
% plot([0:0.05:0.5],ori_rec_rate_hybrid,'--r')
% 
% %errorbar([0:0.05:0.5],orientations_mean,orientations_std_dev)
% legend('Rabbani et al.','Ours')
% 
% xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
% ylabel('correct orientation rate (% of cylinder surface points)','Interpreter','LaTex','FontSize',fontsize);
% set(gca,'XTick',[0:0.1:1.0])
% 
% axis ([-0.01 x_upper_limit 0 101]) 
% set(gca,'fontsize',fontsize);
% 
% export_fig orientation_rec -pdf
%% radius
radius_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_clutter_0.txt';
%radius_file_1='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_clutter_1.txt';
radius_file_hybrid='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_clutter_1.txt';

formatSpec = '%f';
sizeA = [Inf iterations];
%A = fscanf(fileID,formatSpec,sizeA)
radii = importdata(radius_file);
radii_mean = mean(radii,2);
radii_std_dev=std(radii,0,2);

% radii_1 = importdata(radius_file_1);
% radii_mean_1 = mean(radii_1,2);
% radii_std_dev_1=std(radii_1,0,2);

radii_hybrid = importdata(radius_file_hybrid);
radii_mean_hybrid = mean(radii_hybrid,2);
radii_std_dev_hybrid=std(radii_hybrid,0,2);

% PLOTS
figure(3)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(radii_mean)
hold on
errorbar(100*[0:0.5:2.0],radii_mean,radii_std_dev,'--b')
%errorbar([0:0.05:0.5],radii_mean_1,radii_std_dev_1,'k')
errorbar(100*[0:0.5:2.0],radii_mean_hybrid, radii_std_dev_hybrid,'k')

legend('Rabbani et al.','Ours')

xlabel('clutter (% of cylinder surface points)','Interpreter','LaTex','FontSize',fontsize);
ylabel('absolute radius error (m)','Interpreter','LaTex','FontSize',fontsize);

set(gca,'XTick',100*[0:0.5:2.0])

axis ([-1 200 -0.001 0.05]) 
set(gca,'fontsize',fontsize);

scale = 0.1;
pos = get(gca, 'Position');
pos(2) = pos(2)+scale*pos(4);
pos(4) = (1-scale)*pos(4);
set(gca, 'Position', pos)

export_fig clutter_radii_error -pdf

%% RADIUS REC
% radius_rec=radii;
% radius_rec(radii<=0.01)=1
% radius_rec(radii>0.01)=0
% rad_rec_rate=100.0*sum(radius_rec,2)/iterations;
% 
% radius_rec_1=radii_1;
% radius_rec_1(radii_1<=0.01)=1
% radius_rec_1(radii_1>0.01)=0
% rad_rec_rate_1=100.0*sum(radius_rec_1,2)/iterations;
% 
% radius_rec_hybrid=radii_hybrid;
% radius_rec_hybrid(radii_hybrid<=0.01)=1
% radius_rec_hybrid(radii_hybrid>0.01)=0
% rad_rec_rate_hybrid=100.0*sum(radius_rec_hybrid,2)/iterations;

% figure(4)
% fontsize=15;
% set(gcf, 'Color', [1,1,1]);
% hold on
% plot([0:0.05:0.5],rad_rec_rate,'--b')
% plot([0:0.05:0.5],rad_rec_rate_1,'k')
% plot([0:0.05:0.5],rad_rec_rate_hybrid,'--r')
% 
% legend('Normals','Curvature','Hybrid')
% 
% %errorbar([0:0.05:0.5],orientations_mean,orientations_std_dev)
% 
% xlabel('error standard deviation (% of cylinder radius)','Interpreter','LaTex','FontSize',fontsize);
% ylabel('correct radius rate (% of cylinder surface points)','Interpreter','LaTex','FontSize',fontsize);
% set(gca,'XTick',[0:0.1:1.0])
% 
% axis ([-0.01 x_upper_limit 0 101]) 
% set(gca,'fontsize',fontsize);
% 
% export_fig radii_rec -pdf

%% position
position_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position_clutter_0.txt';
%position_file_1='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position_clutter_1.txt';
position_file_hybrid='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position_clutter_1.txt';

formatSpec = '%f';
sizeA = [Inf iterations];
%A = fscanf(fileID,formatSpec,sizeA)
position = importdata(position_file);
position_mean = mean(position,2);
position_std_dev=std(position,0,2);

% position_1 = importdata(position_file_1);
% position_mean_1 = mean(position_1,2);
% position_std_dev_1=std(position_1,0,2);

position_hybrid = importdata(position_file_hybrid);
position_mean_hybrid = mean(position_hybrid,2);
position_std_dev_hybrid=std(position_hybrid,0,2);

% PLOTS
figure(4)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(radii_mean)
hold on
errorbar(100*[0:0.5:2.0],position_mean,position_std_dev,'--b')
%errorbar([0:0.05:0.5],position_mean_1,position_std_dev_1,'k')
errorbar(100*[0:0.5:2.0],position_mean_hybrid, position_std_dev_hybrid,'k')

legend('Rabbani et al.','Ours')

xlabel('clutter (% of cylinder surface points)','Interpreter','LaTex','FontSize',fontsize);
ylabel('absolute position error (m)','Interpreter','LaTex','FontSize',fontsize);

set(gca,'XTick',100*[0:0.5:2.0])

axis ([-1 200 -0.001 2.0])

set(gca,'fontsize',fontsize);

scale = 0.1;
pos = get(gca, 'Position');
pos(2) = pos(2)+scale*pos(4);
pos(4) = (1-scale)*pos(4);
set(gca, 'Position', pos)

export_fig clutter_position_error -pdf








