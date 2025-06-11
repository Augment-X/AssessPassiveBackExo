


vicon = ViconNexus();
Subject = vicon.GetSubjectNames();   %%%% exo no-exo
cameraRate = vicon.GetFrameRate();   %Hz

markers = vicon.GetMarkerNames( Subject{1} ); %% name of markers
Joints  = vicon.GetJointNames( Subject{1} );
markers_count = length(markers);              %% count of markers
[boxX, boxY, boxZ, valid]=vicon.GetTrajectory(Subject{1},markers{1});
time    = (1:length(boxX))/cameraRate ;       %% second
position = zeros(markers_count,length(boxX));

for i=1:markers_count
    [position(3*i-2,:), position(3*i-1,:), position(3*i,:), valid]=vicon.GetTrajectory(Subject{1},markers{i});
end
%%
marker_position_0 = position';

marker_position = smoothdata(marker_position_0,'rlowess',100);

for j=1:markers_count
    eval(['marker_',num2str(j),'(:,1)=marker_position(:,3*',num2str(j),'-2);']);
    eval(['marker_',num2str(j),'(:,2)=marker_position(:,3*',num2str(j),'-1);']);
    eval(['marker_',num2str(j),'(:,3)=marker_position(:,3*',num2str(j),');']);
end

%% 
% sm_window = 500; %% smooth parameter
a = 100;  %% removing offset
b = 250;  %% removing offset
%% fixed 

angle_degree_fixed_frame = rot_x_1(marker_1,marker_2);   % y/z   the angle will be used
angle_degree_fixed_frame_no_offset = remove_off(angle_degree_fixed_frame,a,b);
angle_rad_fixed_frame              = [angle_degree_fixed_frame]/360*2*pi;
angle_rad_fixed_frame_no_offset    = remove_off(angle_rad_fixed_frame,a,b);

%% chest beam

angle_degree_chest_beam = rot_x(marker_8,marker_9);   % y/z   the angle will be used
angle_degree_chest_beam_no_offset = remove_off(angle_degree_chest_beam,a,b);
angle_rad_chest_beam              = [angle_degree_chest_beam]/360*2*pi;
angle_rad_chest_beam_no_offset    = remove_off(angle_rad_chest_beam,a,b);

%% chest_doll 

angle_degree_chest_doll = rot_x(marker_12,marker_13);   % y/z   the angle will be used
angle_degree_chest_doll_no_offset = remove_off(angle_degree_chest_doll,a,b);
angle_rad_chest_doll              = [angle_degree_chest_doll]/360*2*pi;
angle_rad_chest_doll_no_offset    = remove_off(angle_rad_chest_doll,a,b);

%% hip

angle_degree_lower_back_joint = rot_x(marker_17,marker_16);   % y/z   the angle will be used

% angle_degree_lower_back_joint = rot_x(marker_25,marker_24);     % y/z   the angle will be used


angle_degree_lower_back_joint =  smoothdata(angle_degree_lower_back_joint,'rlowess',500);

angle_degree_lower_back_joint_no_offset = remove_off(angle_degree_lower_back_joint,a,b);
angle_rad_lower_back_joint              = [angle_degree_lower_back_joint]/360*2*pi;
angle_rad_lower_back_joint_no_offset    = remove_off(angle_rad_lower_back_joint,a,b);

save('no_exo_angle_degree_lower_back_joint',"angle_degree_lower_back_joint");
%% left leg

% angle_degree_left_leg = rot_x(marker_22,marker_21);   % y/z   the angle will be used

 % angle_degree_left_leg = rot_x(marker_7,marker_6);

angle_degree_left_leg = angle_cal(marker_22,marker_21);   % y/z   the angle will be used
angle_degree_left_leg = smoothdata(angle_degree_left_leg,'rlowess',500);

angle_degree_left_leg_no_offset = remove_off(angle_degree_left_leg,a,b);
angle_degree_left_leg_no_offset_sm = smoothdata(angle_degree_left_leg_no_offset,'rlowess',500);
angle_rad_left_leg              = [angle_degree_left_leg]/360*2*pi;
angle_rad_left_leg_no_offset    = remove_off(angle_rad_left_leg,a,b);

%% right leg

% angle_degree_right_leg = rot_x(marker_24,marker_27);   % y/z   the angle will be used

% angle_degree_right_leg = rot_x(marker_4,marker_1);

angle_degree_right_leg   = angle_cal(marker_27,marker_24);   % y/z   the angle will be used
angle_degree_right_leg   =  smoothdata(angle_degree_right_leg,'rlowess',500);

angle_degree_right_leg_no_offset = remove_off(angle_degree_right_leg,a,b);
angle_degree_right_leg_no_offset_sm = smoothdata(angle_degree_right_leg_no_offset,'rlowess',500);

angle_rad_right_leg              = [angle_degree_right_leg]/360*2*pi;
angle_rad_right_leg_no_offset    = remove_off(angle_rad_right_leg,a,b);

%% bending angle
angle_degree_bending           = (angle_degree_left_leg + angle_degree_right_leg)/2;
angle_degree_bending_no_offset = angle_degree_left_leg_no_offset + angle_degree_right_leg_no_offset;
angle_rad_bending_no_offset    = angle_degree_bending_no_offset/360*2*pi;

save('no_exo_angle_degree_bending',"angle_degree_bending");

%% hip_joint position  lower back joint position

hip_position_L = [marker_17(:,2),marker_17(:,3)];  %% (y,z)
hip_position_R = [marker_18(:,2),marker_18(:,3)]; 

No_exo_hip_Joint_position = [hip_position_L hip_position_R] ;

save('No_exo_hip_Joint_position',"No_exo_hip_Joint_position");

%% hip_leg

angle_degree_hip_leg = angle_degree_lower_back_joint - (angle_degree_left_leg + angle_degree_right_leg)/2 ;
angle_rad_hip_leg    = angle_degree_hip_leg/360*2*pi;

%% figure with offset

figure(); plot(time,angle_degree_fixed_frame);grid on; title('Angles of Body Segments in the Movement'); xlabel('Time(s)'); ylabel('Angle(°)');
hold on; 
plot(time,angle_degree_chest_beam);
plot(time,angle_degree_chest_doll); 
plot(time,angle_degree_lower_back_joint);
plot(time,angle_degree_left_leg);
plot(time,angle_degree_right_leg);
legend('fixed frame','chest beam','chest doll','lower back joint','left leg','right leg');
hold off;

%% figure without offset

figure(); plot(time,angle_degree_fixed_frame_no_offset);grid on; title('Angles of Body Segments in the Movement'); xlabel('Time(s)'); ylabel('Force(°)');
hold on; 
plot(time,angle_degree_chest_beam_no_offset);
plot(time,angle_degree_chest_doll_no_offset); 
plot(time,angle_degree_lower_back_joint_no_offset);
plot(time,angle_degree_left_leg_no_offset);
plot(time,angle_degree_right_leg_no_offset);
legend('fixed frame','chest beam','chest doll','lower back joint','left leg','right leg');


%% angle

% field1 = 'fixed_frame';         value1 = angle_degree_fixed_frame;
% field2 = 'chest_beam';          value2 = angle_degree_chest_beam;
% field3 = 'chest_doll';          value3 = angle_degree_chest_doll;
% field4 = 'lower_back_joint';    value4 = angle_degree_lower_back_joint;
% field5 = 'left_leg';            value5 = angle_degree_left_leg;
% field6 = 'right_leg';           value6 = angle_degree_right_leg;
% field7 = 'bending';             value7 = (angle_degree_right_leg + angle_degree_left_leg)/2;
% field8 = 'hip_leg';             value8 = angle_degree_lower_back_joint - (angle_degree_right_leg + angle_degree_left_leg)/2;
% 
% 
% no_exo_angle_degree = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8);

% save('no_exo_angle_degree','no_exo_angle_degree');

%% function

function [B] = remove_off(A,a,b)
  offset     = sum(A(a:b,:))/(b-a+1);
  samples    = ones(size(A));
  offsetMat  = samples .* offset ;
  B          = A - offsetMat ;
end

function [D] = rot_x(E,F)
global sm_window
  slope = (F(:,2)-E(:,2))./(F(:,3)-E(:,3));
  D = 360*atan(slope)/(2*pi);

%   D     = smoothdata(angle,'rloess',sm_window);
end

function [D] = rot_x_1(E,F)
global sm_window
  slope = (F(:,3)-E(:,3))./(F(:,2)-E(:,2));
  D = 360*atan(slope)/(2*pi);
%   D     = smoothdata(angle,'rloess',sm_window);
end
% 
% function [I] = rot_y(G,H)  
% global sm_window
%   slope = (H(:,1)-G(:,1))./(H(:,3)-G(:,3));
%   angle = 360*atan(slope)/(2*pi);
%   I     = smoothdata(angle,'rloess',sm_window);
% end
% 
% function [J] = rot_z(K,L)
% global sm_window
%   slope = (L(:,1)-K(:,1))./(L(:,2)-K(:,2));
%   angle = 360*atan(slope)/(2*pi);
%   J     = smoothdata(angle,'rloess',sm_window);
% end
% 
% function [M] = rot_z_1(N,O)
% global sm_window
%   slope = (O(:,2)-N(:,2))./(O(:,1)-N(:,1));
%   angle = 360*atan(slope)/(2*pi);
%   M     = smoothdata(angle,'rloess',sm_window);
% end

function [Q] = sm(R)
global sm_window
Q  = smoothdata(R,'rloess',sm_window);
end


function [C] = angle_cal(A,B)

y1 = A(:, 2); % y-coordinates of marker_1
z1 = A(:, 3); % z-coordinates of marker_1
y2 = B(:, 2); % y-coordinates of marker_2
z2 = B(:, 3); % z-coordinates of marker_2

vec_angle = [z2 - z1, y2 - y1];
reference_vector = [1, 0];
C = zeros(size(y1));

for i = 1:length(y1)
    u = vec_angle(i, :); % Current leg vector in frame i
    v = reference_vector; % Reference vector
    
    % Normalize the vectors
    u = u / norm(u);
    v = v / norm(v);
    
    % Compute dot product and angle
    dot_product = dot(u, v);
    angle = acos(dot_product); % Angle in radians (0 to pi)
    
    % Determine the direction of rotation using cross product
    % cross_prod = u(1) * v(2) - u(2) * v(1); % 2D cross product
    % if cross_prod < 0
    %     angle = -angle; % Negative angle for clockwise rotation
    % end
    % 
    % Convert angle to degrees

    C(i) = rad2deg(angle);
end
    C = C - C(1);
% Ensure all angles are positive (optional, if needed)
% C = mod(rotation_angles, 360);

end