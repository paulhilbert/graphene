clear all; clf; clc;
load pointset.dat;
load medoids.dat;
load cluster.dat;
%load clustersteps.dat;
%load medoidsteps.dat;

pointsPerCluster = 200;

n = size(pointset, 1);
numCluster = size(medoids, 1);

centroids = pointset(1:(pointsPerCluster+1):n, :);
clusterColor = cluster .* (0.7 / numCluster);

colormap(jet);
cmap = colormap;
r = interp1(1:size(cmap,1), cmap(:,1), 1:1/8:64);
g = interp1(1:size(cmap,1), cmap(:,2), 1:1/8:64);
b = interp1(1:size(cmap,1), cmap(:,3), 1:1/8:64);
cmap = [r' g' b'];

hold on;
for c=1:numCluster
    cPoints = pointset(cluster == c,:);
    cColor = cmap(c,:);
    plot(cPoints(:,1), cPoints(:,2), '.', 'Color', cColor);
end
%plot(centroids(:,1), centroids(:,2), 's', 'Color', 'm');
plot(pointset(medoids,1), pointset(medoids,2), '*', 'Color', 'r');
hold off;
    
%%

numIterations = size(clustersteps, 2);
for i=numIterations:numIterations
    hold on;
    for c=1:numCluster
        cPoints = pointset(clustersteps(:,i) == c,:);
        cColor = cmap(c,:);
        plot(cPoints(:,1), cPoints(:,2), '.', 'Color', cColor);
    end
    %plot(centroids(:,1), centroids(:,2), 's', 'Color', 'm');
    plot(pointset(medoidsteps(:,i),1), pointset(medoidsteps(:,i),2), '*', 'Color', 'r');
    hold off;
    pause;
  %  clf;
end