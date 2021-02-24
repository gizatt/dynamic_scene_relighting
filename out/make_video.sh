ffmpeg -y -r 5 -start_number 5 -i curr_color_%03d.png  -frames:v 40  output_color.mp4
ffmpeg -y -r 5 -start_number 5 -i curr_brightness_%03d.png -frames:v 40  output_brightness.mp4

