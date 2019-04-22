function [rgb_frames, depth_frames, init_bb] = load_video(video_path)
ground_truth = dlmread([video_path '/init.txt']);

init_bb = ground_truth(1,:);
init_bb = init_bb + 1;

rgb_frames_dir = dir([video_path '/color/']);
depth_frames_dir = dir([video_path '/depth/']);

rgb_frames = cell(length(rgb_frames_dir) - 2, 1);
depth_frames = cell(length(rgb_frames_dir) - 2, 1);

for i=3:length(rgb_frames_dir)
    rgb_frames{i-2} = [video_path 'color/' rgb_frames_dir(i).name];
    depth_frames{i-2} = [video_path 'depth/' depth_frames_dir(i).name];
end

end

