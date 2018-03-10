robot0=importdata('results12_map_robot0.log');
robot1=importdata('results12_map_robot1.log');
robot2=importdata('results12_map_robot2.log');
robot3=importdata('results12_map_robot3.log');
globalmap=importdata('results12_map_globalmap.log');

robot0(:,2)=robot0(:,2)-robot0(1,2);
robot1(:,2)=robot1(:,2)-robot1(1,2);
robot2(:,2)=robot2(:,2)-robot2(1,2);
robot3(:,2)=robot3(:,2)-robot3(1,2);
globalmap(:,2)=globalmap(:,2)-globalmap(1,2);

%plot(globalmap(:,2),globalmap(:,1),'m-.',robot0(:,2),robot0(:,1),'b-.',robot1(:,2),robot1(:,1),'r-.',robot2(:,2),robot2(:,1),'g-.',robot3(:,2),robot3(:,1),'k-.');


robot0=importdata('results12_path_robot0.log');
robot1=importdata('results12_path_robot1.log');
robot2=importdata('results12_path_robot2.log');
robot3=importdata('results12_path_robot3.log');

robot0(:,2)=robot0(:,2)-robot0(1,2);
robot1(:,2)=robot1(:,2)-robot1(1,2);
robot2(:,2)=robot2(:,2)-robot2(1,2);
robot3(:,2)=robot3(:,2)-robot3(1,2);


plot(robot0(:,2),robot0(:,1),'b-.',robot1(:,2),robot1(:,1),'r-.',robot2(:,2),robot2(:,1),'g-.',robot3(:,2),robot3(:,1),'k-.');
