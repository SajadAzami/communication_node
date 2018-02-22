robot0=importdata('results10_map_robot0.log');
robot1=importdata('results10_map_robot1.log');
robot2=importdata('results10_map_robot2.log');
robot3=importdata('results10_map_robot3.log');
globalmap=importdata('results10_map_globalmap.log');

robot0(:,2)=robot0(:,2)-robot0(1,2);
robot1(:,2)=robot1(:,2)-robot1(1,2);
robot2(:,2)=robot2(:,2)-robot2(1,2);
robot3(:,2)=robot3(:,2)-robot3(1,2);
globalmap(:,2)=globalmap(:,2)-globalmap(1,2);

robot0(:,1)=robot0(:,1)*10;
robot1(:,1)=robot1(:,1)*10;
robot2(:,1)=robot2(:,1)*10;
robot3(:,1)=robot3(:,1)*10;
globalmap(:,1)=globalmap(:,1)*10;

%plot(globalmap(:,2),globalmap(:,1),'m-.',robot0(:,2),robot0(:,1),'b-.',robot1(:,2),robot1(:,1),'r-.',robot2(:,2),robot2(:,1),'g-.',robot3(:,2),robot3(:,1),'k-.');


robot0=importdata('results10_path_robot0.log');
robot1=importdata('results10_path_robot1.log');
robot2=importdata('results10_path_robot2.log');
robot3=importdata('results10_path_robot3.log');

robot0(:,2)=robot0(:,2)-robot0(1,2);
robot1(:,2)=robot1(:,2)-robot1(1,2);
robot2(:,2)=robot2(:,2)-robot2(1,2);
robot3(:,2)=robot3(:,2)-robot3(1,2);


plot(robot0(:,2),robot0(:,1),'b-.',robot1(:,2),robot1(:,1),'r-.',robot2(:,2),robot2(:,1),'g-.',robot3(:,2),robot3(:,1),'k-.');
