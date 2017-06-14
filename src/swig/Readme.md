# Important notice
please make sure both
1. DBOW
2. ORB_SLAM
3. this swig warper
are both compiled with the same OpenCV version. 
To do so, run both projects with the same following command: 
```
cmake .. -DOpenCV_DIR:=/home/nubot/data/software/anaconda2/share/OpenCV
```
Otherwise you may encounter problems like:
- "Failed to open settings file at ..."
- Segment fault
