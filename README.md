# rqt_plotbag

This is a modified version of rqt_plot that reads directly from a bag.

Installation:
* `git clone` this repo into your ros workspace/src/rqt_plotbag
* `git checkout` the plotbag branch
* `catkin build`
* `source` ros workspace/devel/setup.bash
* start a `roscore` if not already have one running on your laptop (I don't use it for anything, but rqt requires it still)
* run `rqt_plotbag`

Usage:
1. Click 'Browse' (or paste a full path to a bag file)
2. Click 'Load' to open the bag file
3. Specify the time range you want to plot (shorter ranges will plot quicker)
4. Add topics in the upper left (same as normal rqt_plot)
5. If you change the time range after adding topics, click 'Redraw' to re-plot all added topics

Tip:
If you are plotting multiple topics, disable autodraw, add all the topics you want to plot, set the time range to the range you are interested in, then click redraw. 
This will plot all the topics in parallel which is quicker then adding them one by one.

Tip 2:
You can quickly load a bag and plot topics from a commandline, call it like this:
`rqt_plotbag /path/to/bagfile.bag /obd/speed /obd/accel /obd/brake /obd/torque`
And it will load the bag file and immediately plot those topics over the entire bag.

![screenshot](https://raw.githubusercontent.com/StarskyRobotics/rqt_plot/plotbag/image.png)
