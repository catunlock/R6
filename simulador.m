function T06 = simulador(angles)
%
% This function reads file 'linkdata.mat' containing the 3D models of the
% links of a PUMA560 and plots it with the joint angles given by 'angles'. 
% It returns the transformation that goes from the world reference frame to
% the tool reference frame.
%
% Author: F. Thomas (ESAII-UPC)
% Data: 02/28/2017
%
 
persistent linkdata;

[linkdata] = load('linksdata.mat','s1','s2', 's3','s4','s5','s6','s7','A1');

range = [-2000 3000 -2000 2000 -2000 2000];

% Link parameters and joint angles

a2 = 650;
a3 = 0;
d3 = 190;
d4 = 600;

%     Angle    Range                Default Name
%     Theta 1: 320 (-160 to 160)    90      Waist Joint
%     Theta 2: 220 (-110 to 110)    -90     Shoulder Joint
%     Theta 3: 270 (-135 to 135)    -90     Elbow Joint
%     Theta 4: 532 (-266 to 266)    0       Wrist Roll
%     Theta 5: 200 (-100 to 100)    0       Wrist Bend
%     Theta 6: 532 (-266 to 266)    0       Wrist Swival

t = angles*pi/180; 

% Forward Kinematics

T00 = transl(1000,-1000,1250);
T01 = T00 * trotz(t(1));
T12 = trotx(-pi/2)*trotz(t(2));
T23 = transl(a2,0,d3)*trotz(t(3));

T34 = trotx(-pi/2)*transl(a3,0,d4)*trotz(t(4));
T45 = trotx(pi/2)*trotz(t(5));
T56 = trotx(-pi/2)*trotz(t(6));



T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

Link1 = linkdata.s1.V1*T00';
Link2 = linkdata.s2.V2*T01';
Link3 = linkdata.s3.V3*T02';
Link4 = linkdata.s4.V4*T03';
Link5 = linkdata.s5.V5*T04';
Link6 = linkdata.s6.V6*T05';
Link7 = linkdata.s7.V7*T06';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
hold on;

persistent L1 L2 L3 L4 L5 L6 L7;
persistent h;
if isempty(L1) 
       L1 = patch('Faces', linkdata.s1.F1,...
           'Vertices', Link1(:,1:3),...
           'facec', [0.717,0.116,0.123],...
           'EdgeColor','none');   
       L2 = patch('Faces', linkdata.s2.F2,...
           'Vertices', Link2(:,1:3),...
           'facec', [0.216,1,0.583],...
           'EdgeColor','none');
        L3 = patch('faces', linkdata.s3.F3,...
           'vertices', Link3(:,1:3),...
           'facec', [0.306,0.733,1],...
           'EdgeColor','none');
        L4 = patch('faces', linkdata.s4.F4,...
           'vertices', Link4(:,1:3),...
           'facec', [1,0.542,0.493],...
           'EdgeColor','none');
        L5 = patch('faces', linkdata.s5.F5,...
           'vertices' ,Link5(:,1:3),...
           'facec', [0.216,1,.583],...
           'EdgeColor','none');
        L6 = patch('faces', linkdata.s6.F6,...
           'vertices' ,Link6(:,1:3),...
           'facec', [1,1,0.255],...
           'EdgeColor','none');
        L7 = patch('faces', linkdata.s7.F7,...
           'vertices' ,Link7(:,1:3),...
           'facec', [1,1,0.255],...
           'EdgeColor','none');
       

        h = trplot(eye(4),'length', 550, 'arrow');
        trplot(h, T06,'length', 550, 'arrow');

        grid on;
        axis(range);
        daspect([1 1 1]);
        view(3); 
        camlight;
        lighting gouraud;
else
        set(L1, 'Vertices', Link1(:,1:3))
        set(L2, 'Vertices', Link2(:,1:3))
        set(L3, 'Vertices', Link3(:,1:3))
        set(L4, 'Vertices', Link4(:,1:3))
        set(L5, 'Vertices', Link5(:,1:3))
        set(L6, 'Vertices', Link6(:,1:3))
        set(L7, 'Vertices', Link7(:,1:3))
        trplot(h, eye(4), 'length', 550, 'arrow');
        trplot(h, T06);
        
        %disp( T06(:,4))
        
        plot3(T06(1,4),T06(2,4),T06(3,4), '*m');
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%