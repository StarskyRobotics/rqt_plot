# rqt_plotbag

This is a modified version of rqt_plot that reads directly from a bag.

Usage:
1. Click 'Browse' (or paste a full path to a bag file)
2. Click 'Load' to open the bag file
3. Specify the time range you want to plot (shorter ranges will plot quicker)
4. Add topics in the upper left (same as normal rqt_plot)
5. If you change the time range after adding topics, click 'Redraw' to re-plot all added topics

Tip:
If you are plotting multiple topics, first set the time range to really short (eg make start & end the same time), 
then add all the topics you want, then set the time range to the range you are interested in, then click redraw. 
This will plot all the topics in parallel which is quicker then adding them one by one.
