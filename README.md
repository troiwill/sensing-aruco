# sensing-aruco
A library for sensing and working with ArUco markers in simulated and physical environments.

Lib request:
sys 
numpy
pyzed.sl
cv2
pandas
matplotlib_pyplot

cd to YOUR_LOCATION
python3 main.py

Note that in if __name__ == "__main__": have two function, one is read_AR_marker() and the other is main()
read_AR_marker() is used to read the image in the current location and compute its (x,y).
main() is used to open the camera and detect the aruco marker continuounly and also compute its (x,y).


