%% First add path to right folder
%  https://github.com/moulli/balancoire_matlab_files
%  then add path to folder: bouts_detection_manual

addpath('/home/ljp/Science/balancoire_data/balancoire_matlab_files/bouts_detection_manual')


%% Give link to the video you want to analyze

vidpath = '/home/ljp/Desktop/geoffrey_tail.avi';


%% Define parameters for tail tracking

num_segments = 9;
inertia = 0.5;
body_length = 0;
tail_length = 140;
initial_box = 1.2;
box_increment = 0.01;


%% The following functions are interactive
%  When you launch then, you are asked to select two different points, one
%  after the other. The first point should be the closest possible to the
%  head. But it should be on the tail, and not on the structure that is
%  supporting the fish! The second point should be further on the tail, in
%  the direction of it. The direction vector between the first and the
%  second point will be where the algorithm looks for the first segment.


%% If you want to visualize tracking, and adapt parameters

trackingPlot_manual(vidpath, 'num_segments', num_segments, 'inertia', inertia, ...
                    'body_length', body_length, 'tail_length', tail_length, 'initial_box', ...
                    initial_box, 'box_increment', box_increment); 
                
                
%% This one is the most important
%  It returns a structure with all the tracking data
                
tracking = boutsWrapper_manual(vidpath, 'num_segments', num_segments, 'inertia', inertia, ...
                               'body_length', body_length, 'tail_length', tail_length, 'initial_box', ...
                               initial_box, 'box_increment', box_increment);

                           
%% If you want to save the tracking in a new video

saveTracking_manual(vidpath, 'num_segments', num_segments, 'inertia', inertia, ...
                    'body_length', body_length, 'tail_length', tail_length, 'initial_box', ...
                    initial_box, 'box_increment', box_increment); 
                
