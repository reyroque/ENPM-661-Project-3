# ENPM-661-Project3
Project 3 of ENPM-661 Planning For Autonomous Robots Course
# Written by: Rey Roque-Pérez and Dustin Hartnett
# UID: reyroque: 120498626
# UID: dhartnet: 115262249
Project 3 Part 1 of ENPM-661 Planning For Autonomous Robots Course

Implementation of the A* Algorithm for a Point Robot

# How to Run
Run the program by double clicking it or by opening it in an IDE.
Enter the clearance from obstacles, the step size, start and goal coordinates as prompted.
The code will check that the provided coordinates are within the workspace and not inside any obstacles.

The clearance is how far from obstacles and the edge of the map the robot will stay  away from. The step size is the distance between nodes to be checked.

# Dependencies
The following Python libraries are used:

opencv-python: For visualizing the workspace and creating a video.

queue: For using the PriorityQueue function

numpy: For resizing the arrays used to draw the obstacles with opencv.

time: For calculating the duration of computation

# Output
A video file called a_star_output.mp4 will be generated in the same directory were the code is ran from after it is done running.

# Github Repository: 
https://github.com/reyroque/ENPM-661-Project-3 
