result1=importdata('./dataset_mwm/results1_map.log');
result2=importdata('./dataset_mwm/results2_map.log');
result3=importdata('./dataset_mwm/results3_map.log');
result4=importdata('./dataset_mwm/results4_map.log');
result5=importdata('./dataset_mwm/results5_map.log');
seconds=result5(:,2);
seconds=reshape(seconds,[5,13]);
mwm_data_table=horzcat(result1(:,1),result2(:,1),result3(:,1),result4(:,1),result5(:,1));
mwm_mean_of_data=mean(mwm_data_table,2);
mwm_mean_of_data=reshape(mwm_mean_of_data,[5,13]);
mwm_standard_deviation = std(mwm_data_table,0,2);
mwm_standard_deviation=reshape(mwm_standard_deviation,[5,13]);


result6=importdata('./dataset_direct/results6_map.log');
result7=importdata('./dataset_direct/results7_map.log');
result8=importdata('./dataset_direct/results8_map.log');
result9=importdata('./dataset_direct/results9_map.log');
result10=importdata('./dataset_direct/results10_map.log');
direct_data_table=horzcat(result6(:,1),result7(:,1),result8(:,1),result9(:,1),result10(:,1));
direct_mean_of_data=mean(direct_data_table,2);
direct_mean_of_data=reshape(direct_mean_of_data,[5,13]);
direct_standard_deviation = std(direct_data_table,0,2);
direct_standard_deviation=reshape(direct_standard_deviation,[5,13]);


result11=importdata('./dataset_osm/results11_map.log');
result12=importdata('./dataset_osm/results12_map.log');
result13=importdata('./dataset_osm/results13_map.log');
result14=importdata('./dataset_osm/results14_map.log');
result15=importdata('./dataset_osm/results15_map.log');
osm_data_table=horzcat(result11(:,1),result12(:,1),result13(:,1),result14(:,1),result15(:,1));
osm_mean_of_data=mean(osm_data_table,2);
osm_mean_of_data=reshape(osm_mean_of_data,[5,13]);
osm_standard_deviation = std(osm_data_table,0,2);
osm_standard_deviation=reshape(osm_standard_deviation,[5,13]);



result16=importdata('./dataset_swm/results16_map.log');
result17=importdata('./dataset_swm/results17_map.log');
result18=importdata('./dataset_swm/results18_map.log');
result19=importdata('./dataset_swm/results19_map.log');
result20=importdata('./dataset_swm/results20_map.log');
swm_data_table=horzcat(result16(:,1),result17(:,1),result18(:,1),result19(:,1),result20(:,1));
swm_mean_of_data=mean(swm_data_table,2);
swm_mean_of_data=reshape(swm_mean_of_data,[5,13]);
swm_standard_deviation = std(swm_data_table,0,2);
swm_standard_deviation=reshape(swm_standard_deviation,[5,13]);




figure;
hold on;
errorbar(seconds(1,:),mwm_mean_of_data(1,:),mwm_standard_deviation(1,:)/2,'-s','LineWidth' ,0.7,'LineStyle' ,':','Color','blue' ,'Marker','diamond','MarkerSize',5, 'MarkerEdgeColor','blue','MarkerFaceColor','blue');
errorbar(seconds(1,:),direct_mean_of_data(1,:),direct_standard_deviation(1,:)/2,'-s','LineWidth' ,0.7,'LineStyle' ,'--','Color','red'  ,'Marker','*','MarkerSize',5, 'MarkerEdgeColor','red','MarkerFaceColor','red');
errorbar(seconds(1,:),osm_mean_of_data(1,:),osm_standard_deviation(1,:)/2,'-s','LineWidth' ,0.7,'LineStyle' ,'-','Color','black'  ,'Marker','+','MarkerSize',5, 'MarkerEdgeColor','black','MarkerFaceColor','black');
errorbar(seconds(1,:),swm_mean_of_data(1,:),swm_standard_deviation(1,:)/2,'-s','LineWidth' ,0.7,'LineStyle' ,'-.','Color','green' ,'Marker','x','MarkerSize',5, 'MarkerEdgeColor','green','MarkerFaceColor','green');
