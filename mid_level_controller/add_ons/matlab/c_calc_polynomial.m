clear all;
speed_obj = rossubscriber('/speed');
accln_obj = rossubscriber('/accl_in');
c_matrix=zeros(100,1);
vel_sum=zeros(100,1);
counter_matrix=zeros(100,1);
try
    while 1
        speeddata = receive(speed_obj,10);
        acclndata = receive(accln_obj,10);
        current_speed = speeddata.Data;
        current_accln = acclndata.Data;
        current_accln = double(current_accln);
        vel_sum(current_accln+1) = vel_sum(current_accln+1) + current_speed ;
        counter_matrix(current_accln+1) = counter_matrix(current_accln+1) + 1 ;
    end
catch
end
y = vel_sum ./counter_matrix;
final_c=[];
x = [];
for i=1:100
    if (isnan(y(i))==0)
        if (i<=12)
            x = [x;i] ;
            final_c = [final_c;y(i)];
        end
        if ( i>12 && y(i) ~=0 )
            x = [x;i] ;
            final_c = [final_c;y(i)];
        end
    end
end
figure(1);
plot(final_c,x);
nth_order = 5;
coeffecients = polyfit(final_c,x,nth_order);
y_new=[];
input_vel=0:25;
outlput_pedal=polyval(coeffecients,input_vel);
hold on;
plot(input_vel,outlput_pedal);
ylabel('Pedal Press(Percentage)');
xlabel('velocity(kmph)');
