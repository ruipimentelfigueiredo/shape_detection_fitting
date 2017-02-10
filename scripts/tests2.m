iterations=200;


%% orientation
orientation_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_noise.txt';
orientation_file_ours='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_noise_ours.txt';
orientation_file_hybrid='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation_noise_hybrid.txt';

formatSpec = '%f';
sizeA = [Inf 200];
%A = fscanf(fileID,formatSpec,sizeA)
orientations = importdata(orientation_file);
orientations = (180.0/pi)*orientations;
orientations_mean = mean(orientations,2);
orientations_std_dev=std(orientations,0,2);

orientations_ours = importdata(orientation_file_ours);
orientations_ours = (180.0/pi)*orientations_ours;
orientations_ours_mean = mean(orientations_ours,2);
orientations_ours_std_dev=std(orientations_ours,0,2);

orientations_hybrid = importdata(orientation_file_hybrid);
orientations_hybrid = (180.0/pi)*orientations_hybrid;
orientations_hybrid_mean = mean(orientations_hybrid,2);
orientations_hybrid_std_dev=std(orientations_hybrid,0,2);


% PLOTS
close all

% figure(1)
% fontsize=15;
% set(gcf, 'Color', [1,1,1]);
% %plot(orientations_mean)
% hold on
% errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev,'--b')
% errorbar([0:0.05:1.0],orientations_ours_mean,orientations_ours_std_dev,'k')
% errorbar([0:0.05:1.0],orientations_hybrid_mean,orientations_hybrid_std_dev,'--r')
% 
% 
% legend('Rabanni','Ours','Hybrid')
% xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
% ylabel('orientation error (º)','Interpreter','LaTex','FontSize',fontsize);
% set(gca,'XTick',[0:0.1:1.0])
% 
% axis ([-0.01 0.51 0 70]) 
% set(gca,'fontsize',fontsize);
% export_fig orientation_error -pdf

%% ORIENTATIONS REC
orientations_rec=orientations;
orientations_rec(orientations_rec<=12)=1
orientations_rec(orientations_rec>12)=0
ori_rec_rate=100.0*sum(orientations_rec,2)/iterations;

orientations_rec_ours=orientations_ours;
orientations_rec_ours(orientations_rec_ours<=12)=1
orientations_rec_ours(orientations_rec_ours>12)=0
ori_rec_rate_ours=100.0*sum(orientations_rec_ours,2)/iterations

orientations_rec_hybrid=orientations_hybrid;
orientations_rec_hybrid(orientations_rec_hybrid<=12)=1
orientations_rec_hybrid(orientations_rec_hybrid>12)=0
ori_rec_rate_hybrid=100.0*sum(orientations_rec_hybrid,2)/iterations;


figure(2)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
hold on
plot([0:0.05:1.0],ori_rec_rate,'--b')
plot([0:0.05:1.0],ori_rec_rate_ours,'k')
plot([0:0.05:1.0],ori_rec_rate_hybrid,'--r')

%errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev)
legend('Normals','Curvature','Hybrid')

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('orientation recognition rate (%)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.51 0 101]) 
set(gca,'fontsize',fontsize);

export_fig orientation_rec -pdf
%% radius
radius_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_noise.txt';
radius_file_ours='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_noise_ours.txt';
radius_file_hybrid='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius_noise_hybrid.txt';

formatSpec = '%f';
sizeA = [Inf 200];
%A = fscanf(fileID,formatSpec,sizeA)
radii = importdata(radius_file);
radii_mean = mean(radii,2);
radii_std_dev=std(radii,0,2);

radii_ours = importdata(radius_file_ours);
radii_mean_ours = mean(radii_ours,2);
radii_std_dev_ours=std(radii_ours,0,2);

radii_hybrid = importdata(radius_file_hybrid);
radii_mean_hybrid = mean(radii_hybrid,2);
radii_std_dev_hybrid=std(radii_hybrid,0,2);

% PLOTS
% figure(3)
% fontsize=15;
% set(gcf, 'Color', [1,1,1]);
% %plot(radii_mean)
% hold on
% errorbar([0:0.05:1.0],radii_mean,radii_std_dev,'--b')
% errorbar([0:0.05:1.0],radii_mean_ours,radii_std_dev_ours,'k')
% errorbar([0:0.05:1.0],radii_mean_hybrid, radii_std_dev_hybrid,'--r')
% 
legend('Normals','Curvature','Hybrid')
% 
% xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
% ylabel('radius error (m)','Interpreter','LaTex','FontSize',fontsize);
% set(gca,'XTick',[0:0.1:1.0])
% 
% axis ([-0.01 0.5 -0.001 0.05]) 
% set(gca,'fontsize',fontsize);
% 
% export_fig radii_error -pdf

%% RADIUS REC
radius_rec=radii;
radius_rec(radii<=0.01)=1
radius_rec(radii>0.01)=0
rad_rec_rate=100.0*sum(radius_rec,2)/iterations;

radius_rec_ours=radii_ours;
radius_rec_ours(radii_ours<=0.01)=1
radius_rec_ours(radii_ours>0.01)=0
rad_rec_rate_ours=100.0*sum(radius_rec_ours,2)/iterations;

radius_rec_hybrid=radii_hybrid;
radius_rec_hybrid(radii_hybrid<=0.01)=1
radius_rec_hybrid(radii_hybrid>0.01)=0
rad_rec_rate_hybrid=100.0*sum(radius_rec_hybrid,2)/iterations;

figure(4)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
hold on
plot([0:0.05:1.0],rad_rec_rate,'--b')
plot([0:0.05:1.0],rad_rec_rate_ours,'k')
plot([0:0.05:1.0],rad_rec_rate_hybrid,'--r')

legend('Normals','Curvature','Hybrid')

%errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('radius recognition rate (%)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.51 0 101]) 
set(gca,'fontsize',fontsize);

export_fig radii_rec -pdf

%% position
position_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position_noise.txt';
%position_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position_noise_ours.txt';

formatSpec = '%f';
sizeA = [Inf 200];
%A = fscanf(fileID,formatSpec,sizeA)
positions = importdata(position_file);
positions_mean = mean(positions,2);
positions_std_dev=std(positions,0,2);

% PLOTS
figure(3)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(positions_mean)
errorbar([0:0.05:1.0],positions_mean,positions_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('position error (m)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])
axis ([-0.01 0.5 0 0.1]) 
set(gca,'fontsize',fontsize);

%% Combined recognition rates


combined_rec_rate=100.0*sum(orientations_rec&radius_rec,2)/iterations;
combined_rec_rate_ours=100.0*sum(orientations_rec_ours&radius_rec_ours,2)/iterations;
combined_rec_rate_hybrid=100.0*sum(orientations_rec_hybrid&radius_rec_hybrid,2)/iterations;

figure(5)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
hold on
plot([0:0.05:1.0],combined_rec_rate,'--b')
plot([0:0.05:1.0],combined_rec_rate_ours,'k')
plot([0:0.05:1.0],combined_rec_rate_hybrid,'--r')
legend('Normals','Curvature','Hybrid')













