iterations=200;
%% orientation
orientation_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_orientation.txt';

formatSpec = '%f';
sizeA = [Inf 200];
%A = fscanf(fileID,formatSpec,sizeA)
orientations = importdata(orientation_file);
orientations = (180.0/pi)*orientations;
orientations_mean = mean(orientations,2);
orientations_std_dev=std(orientations,0,2);

% PLOTS
close all

figure(1)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(orientations_mean)
errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('orientation error (º)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.51 0 70]) 
set(gca,'fontsize',fontsize);
export_fig orientation_error -pdf

%% ORIENTATIONS REC
orientations_rec=orientations;
orientations_rec(orientations_rec<=12)=1
orientations_rec(orientations_rec>12)=0
ori_rec_rate=100.0*sum(orientations_rec,2)/iterations;
figure(4)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
plot([0:0.05:1.0],ori_rec_rate)
%errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('orientation recognition rate (%)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.51 0 101]) 
set(gca,'fontsize',fontsize);

export_fig orientation_rec -pdf
%% radius
radius_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_radius.txt';

formatSpec = '%f';
sizeA = [Inf 200];
%A = fscanf(fileID,formatSpec,sizeA)
radii = importdata(radius_file);
radii_mean = mean(radii,2);
radii_std_dev=std(radii,0,2);

% PLOTS
figure(2)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
%plot(radii_mean)
errorbar([0:0.05:1.0],radii_mean,radii_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('radius error (m)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.5 0 0.1]) 
set(gca,'fontsize',fontsize);

export_fig radii_error -pdf

%% RADIUS REC
radius_rec=radii;
radius_rec(radii<=0.01)=1
radius_rec(radii>0.01)=0
rad_rec_rate=100.0*sum(radius_rec,2)/iterations;
figure(5)
fontsize=15;
set(gcf, 'Color', [1,1,1]);
plot([0:0.05:1.0],rad_rec_rate)
%errorbar([0:0.05:1.0],orientations_mean,orientations_std_dev)

xlabel('$\sigma$','Interpreter','LaTex','FontSize',fontsize);
ylabel('radius recognition rate (%)','Interpreter','LaTex','FontSize',fontsize);
set(gca,'XTick',[0:0.1:1.0])

axis ([-0.01 0.51 0 101]) 
set(gca,'fontsize',fontsize);

export_fig radii_rec -pdf

%% position
position_file='/home/rui/rosbags/angle_bins_30_radius_bins_10_position_bins_10_position.txt';

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
axis ([-0.01 0.5 0 0.15]) 
set(gca,'fontsize',fontsize);

%% recognition rates

