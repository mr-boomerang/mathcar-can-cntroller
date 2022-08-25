
speed_obj = rossubscriber('/speed');
steer_obj = rossubscriber('/steer_in');
current_steer=0.0;
current_speed=0.0;
Distance=0.0;
angle=0.0;
distance_x=0;
distance_y=0;
position_Data=[0,0];
time_step= 10; %%in ms
while 1
    speeddata = receive(speed_obj,10);
    steerdata = receive(steer_obj,10);
    current_speed = speeddata.Data;
    current_steer = steerdata.Data;
    temp_dist = current_speed * time_step ;
    angle = angle + double(current_steer);
    distance_x = distance_x + temp_dist * cosd(angle);
    distance_y = distance_y + temp_dist * sind(angle);
    position_Data = [position_Data;distance_x,distance_y];
    
end