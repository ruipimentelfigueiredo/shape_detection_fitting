close all
load('gt_squeeze.txt')
load('squeeze.txt')
load('fitting_quality_cylinders_1.txt')
load('fitting_quality_others_1.txt')
test_log=csvread('test_log.csv');
train_log=csvread('train_log.csv');

fontsize=15;

[X1,Y1,T1] = perfcurve(gt_squeeze(:,1),squeeze(:,1),1);

%[X2,Y2,T2] = perfcurve(gt_squeeze(:,2),squeeze(:,2),1);
gt_fitting=[ones(length(fitting_quality_cylinders_1),1); zeros(length(fitting_quality_others_1),1)];
fitting=[fitting_quality_cylinders_1(:,1); fitting_quality_others_1(:,1)];
[X2,Y2,T2] = perfcurve(gt_fitting(:,1),fitting(:,1),1);

plot(X1,Y1,'b')
hold on
plot(X2,Y2,'r')
xlabel('False positive rate','FontSize',fontsize); ylabel('True positive rate','FontSize',fontsize);
legend('SqueezeNet Classifier','Quality of fitting')
%title('ROC Curves for Logistic Regression, SVM, and Naive Bayes Classification')

set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);



%Learning plot
training_iterations=train_log(:,2);
training_loss=train_log(:,4);

testing_iterations=test_log(:,2);
testing_loss=test_log(:,4);

testing_accuracy=test_log(:,5);



export_fig roc_curve -pdf

figure(2)
plot(training_iterations,training_loss,'r');
hold on
plot(testing_iterations,testing_accuracy,'b');
plot(testing_iterations,testing_loss,'g');

xlabel('Iterations','FontSize',fontsize); ylabel('Loss','FontSize',fontsize);

legend('Training loss','Test loss','Test accuracy')

set(gca,'fontsize',fontsize);
set(gcf, 'Color', [1,1,1]);
export_fig training -pdf
