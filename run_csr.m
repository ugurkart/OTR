clc
close all
clear all
warning off all
dbstop if error

PORT = 55000;

visualize = true;
debug = false;

tracker_path = '/home/ugurkart/Codebase/OTR/';

addpath(tracker_path);
addpath(fullfile(tracker_path, 'mex'));
addpath(fullfile(tracker_path, 'utils'));
addpath(fullfile(tracker_path, 'features'));
addpath(fullfile(tracker_path, 'include'));
addpath(fullfile(tracker_path, 'lib'));
addpath(fullfile(tracker_path, 'redetection'));
addpath(fullfile(tracker_path, 'tracker'));
p = genpath(fullfile(tracker_path, 'rgbd-tracker-plugin'));
addpath(p);

root_path = '/home/ugurkart/Datasets/Princeton_RGBD/PTB_Public/';
all_video_dir = dir(root_path);
num_videos = max(size(all_video_dir));

mask_folder_path  = '/masks/';
projections_folder_path = '/projections/';
current_center_file_path = '/center.txt';
current_icp_error_file_path = '/current_icp_error.txt';

icp_error_threshold_file_path = '/icp_error_threshold.txt';

tcp_connection = tcpip('localhost', PORT, 'NetworkRole', 'server');

icp_file = fopen([tracker_path icp_error_threshold_file_path], 'w');
fprintf(icp_file, '%f', 0.0005);
fclose(icp_file);

output_folder = '/home/ugurkart/Desktop/OTR_public/';
mkdir(output_folder);

for i=3:num_videos
    i
    video_name =  all_video_dir(i).name
    video_path = [root_path video_name '/'];
    
    status = system('/home/ugurkart/Codebase/co-fusion-master/build-co-fusion-master-Desktop_Qt_5_7_0_GCC_64bit-Release/GUI/CoFusion &');
    
    fopen(tcp_connection);
    
    if(strcmp(tcp_connection.Status, 'open'))
        fprintf(tcp_connection, 'CONNECT');
        
        while(tcp_connection.BytesAvailable == 0)
        end
        
        received_data = fread(tcp_connection, tcp_connection.BytesAvailable, 'char');
        received_data = char(received_data);
        
        if(strcmp(received_data', 'CONNECT_GOOD'))
            disp('Co-Fusion Connected')
        end
        
        fprintf(tcp_connection, video_name);
    end
    
    mask_folder = [root_path video_name '/' mask_folder_path];
    mkdir(mask_folder);
    
    projections_folder = [root_path video_name '/' projections_folder_path];
    mkdir(projections_folder);
    
    file_name = [output_folder video_name '.txt'];
    fileID = fopen(file_name, 'w');
    
    [rgb_frames, depth_frames, init_rect] = load_video(video_path);
    
    fprintf(fileID,'%f,%f,%f,%f\n', init_rect(1), init_rect(2), init_rect(1) + init_rect(3), init_rect(2) + init_rect(4));
    
    init_rgb_img = imread(rgb_frames{1});
    init_depth_img = uint16(imread(depth_frames{1}));
       
    [rgbd_plugin] = rgbd_plugin_init(init_rgb_img, init_depth_img, init_rect, visualize, debug);
    rgbd_plugin.tcp_connection = tcp_connection;
        
    rgbd_plugin.mask_folder = mask_folder;
    rgbd_plugin.projections_folder = projections_folder;
    rgbd_plugin.icp_error_file = [root_path video_name '/' current_icp_error_file_path];
    rgbd_plugin.icp_error_threshold = 0.0005;
    
    tracker = tracker_init(init_rgb_img, init_rect, rgbd_plugin.mask);
    rgbd_plugin.aspect_ratio_threshold = 0.20;
    rgbd_plugin.prev_aspect_ratio = double(init_rect(3)) / double(init_rect(4));
    
    written_binary_mask = rgbd_plugin.binary_mask;
    written_binary_mask(find(written_binary_mask == 1)) = 1;
    imwrite(uint8(written_binary_mask), [mask_folder 'Mask_' num2str(1, '%08d') '.png']);
    
    fprintf(rgbd_plugin.tcp_connection, "PROCESS_FRAME");
    while(rgbd_plugin.tcp_connection.BytesAvailable == 0)
    end
    received_data = fread(rgbd_plugin.tcp_connection, rgbd_plugin.tcp_connection.BytesAvailable, 'char');
    received_data = char(received_data);
    
    if(strcmp(received_data', 'PROCESS_FRAME_CONFIRMED'))
        disp('Process frame command is received on C++ side');
    end
    
    fprintf(rgbd_plugin.tcp_connection, num2str(1));
    
    if(rgbd_plugin.visualize)
        rgbd_plugin = visualize_all(rgbd_plugin, init_rgb_img, init_rect);
    end
    
    while(rgbd_plugin.tcp_connection.BytesAvailable == 0)
    end
    
    received_data = fread(rgbd_plugin.tcp_connection, rgbd_plugin.tcp_connection.BytesAvailable, 'char');
    received_data = char(received_data);
    
    if(strcmp(received_data', ['Frame_' num2str(1)]))
        disp('Frame processed on C++ side');
    end
    
    for f = 2:length(rgb_frames)
        
        rgbd_plugin.frame_no = f;
        img = imread(rgb_frames{f});
        depth_img = uint16(imread(depth_frames{f}));
        
        rgbd_plugin.depth_img = depth_img;
        
        if(rgbd_plugin.occlusion == false)
            if(mod(f, rgbd_plugin.mdcf_period) == 0)
                [resp_score, c, bb, temp_params, rgbd_plugin, tracker] = tracker_find_best_filter(img, tracker, rgbd_plugin, tracker.c);
            else
                [bb, resp_score, temp_params] = tracker_process_frame(tracker, tracker.c, img, false);
            end
            temp_params.c = single(temp_params.c);
            temp_params.bb = single(temp_params.bb);
            rgbd_plugin.c = temp_params.c;
            rgbd_plugin.bb = temp_params.bb;
            rgbd_plugin.currentScaleFactor = temp_params.currentScaleFactor;
            rgbd_plugin.response(rgbd_plugin.frame_counter) = resp_score;
            
            rgbd_plugin.current_response = resp_score;
            
            tracker.c = temp_params.c;
            tracker.bb = temp_params.bb;
            tracker.currentScaleFactor = temp_params.currentScaleFactor;
            
            if(mod(rgbd_plugin.frame_counter, rgbd_plugin.history) == 0)
                rgbd_plugin.frame_counter = 1;
            else
                rgbd_plugin.frame_counter = rgbd_plugin.frame_counter + 1;
            end
            
            [rgbd_plugin] = rgbd_plugin_process_frame(img, depth_img, temp_params.c, temp_params.bb, rgbd_plugin, tracker);
            
            if(rgbd_plugin.occlusion)
                if(rgbd_plugin.num_occluded_frames == 0)
                    if(max(size(rgbd_plugin.consistent_centers)) == rgbd_plugin.successful_frame_limit) % Valid consistency measure
                        for s=2:rgbd_plugin.successful_frame_limit
                            rgbd_plugin.consistent_velocity(:,:) = rgbd_plugin.consistent_velocity(:,:) + ...
                                abs(rgbd_plugin.consistent_centers(s,:) - rgbd_plugin.consistent_centers(s-1, :));
                        end
                        
                        rgbd_plugin.consistent_velocity = rgbd_plugin.consistent_velocity / rgbd_plugin.successful_frame_limit;
                    end
                end
                fprintf(fileID, '%s,%s,%s,%s\n', 'NaN', 'NaN', 'NaN', 'NaN');
                if(rgbd_plugin.visualize)
                    rgbd_plugin = visualize_all(rgbd_plugin, img, temp_params.bb);
                end
                
                continue;
            else
                fprintf(fileID, '%f,%f,%f,%f\n', tracker.bb(1), tracker.bb(2), tracker.bb(1) + tracker.bb(3), tracker.bb(2) + tracker.bb(4));
                
                [ occlusion_state, rgbd_plugin ] = analyze_mean_response(rgbd_plugin);
                
                if((f > 3) && (rgbd_plugin.use_csr_mask == false) && (occlusion_state == false))
                    aspect_ratio_diff = abs(rgbd_plugin.prev_aspect_ratio - rgbd_plugin.optimal_bb_aspect_ratio);
                    
                    if(aspect_ratio_diff > rgbd_plugin.aspect_ratio_threshold)
                        rgbd_plugin.filters(rgbd_plugin.filter_index, :, :, :) = zeros(size(tracker.H));
                        rgbd_plugin.filters(rgbd_plugin.filter_index, :, : ,:) = tracker.H;
                        rgbd_plugin.filter_index = rgbd_plugin.filter_index + 1;
                        rgbd_plugin.prev_aspect_ratio = rgbd_plugin.optimal_bb_aspect_ratio;
                    end
                end
                current_center_file_id = fopen([root_path video_name current_center_file_path], 'w');
                fprintf(current_center_file_id, '%f %f', tracker.c(1), tracker.c(2));
                fclose(current_center_file_id);
            end
            
            if(rgbd_plugin.visualize)
                rgbd_plugin = visualize_all(rgbd_plugin, img, temp_params.bb);
            end
            
            if(rgbd_plugin.tracker_update_flag)
                [tracker] = tracker_update(tracker, img, temp_params, rgbd_plugin.update_mask);
            end
        else
            rgbd_plugin.num_occluded_frames = rgbd_plugin.num_occluded_frames + 1;
            [rgbd_plugin, c, bb, resp_score, temp_params] = occlusion_recovery(rgbd_plugin, tracker, img);
            
            tracker.c = c;
            tracker.bb = bb;
            
            % set the updated scale factor only if non-occluded
            if rgbd_plugin.use_median_ref
                rgbd_plugin.currentScaleFactor = temp_params.scale_factor;
            else
                rgbd_plugin.currentScaleFactor = temp_params.currentScaleFactor;
            end
            if(rgbd_plugin.occlusion == false)
                rgbd_plugin.response(rgbd_plugin.frame_counter) = resp_score;
                
                if(mod(rgbd_plugin.frame_counter, rgbd_plugin.history) == 0)
                    rgbd_plugin.frame_counter = 1;
                else
                    rgbd_plugin.frame_counter = rgbd_plugin.frame_counter + 1;
                end
                
                current_center_file_id = fopen([root_path video_name current_center_file_path], 'w');
                fprintf(current_center_file_id, '%f %f', tracker.c(1), tracker.c(2));
                fclose(current_center_file_id);
                
                [rgbd_plugin] = rgbd_plugin_process_frame(img, depth_img, c, bb, rgbd_plugin, tracker);
                
                if(rgbd_plugin.occlusion == false)
                    if rgbd_plugin.use_median_ref
                        tracker.currentScaleFactor = temp_params.scale_factor;
                    else
                        tracker.currentScaleFactor = temp_params.currentScaleFactor;
                    end
                    rgbd_plugin.occlusion_recovery_frame_no = rgbd_plugin.frame_no;
                    if(rgbd_plugin.tracker_update_flag)
                        [tracker] = tracker_update(tracker,img, temp_params, rgbd_plugin.update_mask);
                    end
                end
            end
            if(rgbd_plugin.visualize)
                rgbd_plugin = visualize_all(rgbd_plugin, img, bb);
            end
            if(rgbd_plugin.occlusion == false)
                fprintf(fileID, '%f,%f,%f,%f\n', bb(1), bb(2), bb(1) + bb(3), bb(2) + bb(4));
            else
                fprintf(fileID, '%s,%s,%s,%s\n', 'NaN', 'NaN', 'NaN', 'NaN');
            end
        end
    end
    
    fclose(tcp_connection);
    fclose(fileID);
    close all
    
end
