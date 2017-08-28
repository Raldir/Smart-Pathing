import cv2
import argparse
import os
import glob


dir_path = 'Visualisation/visOfSteps/'
ext = 'png'
output = 'output.mp4'

images = {}

for f in os.listdir(dir_path):
    if f.endswith(ext):
        index = int((f.split('.',1)[0]))
        images[index] = f

# Determine the width and height from the first image
image_path = os.path.join(dir_path, images[1])
frame = cv2.imread(image_path)
#cv2.imshow('video',frame)
height, width, channels = frame.shape

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use lower case
out = cv2.VideoWriter(output, fourcc, 20.0, (width, height))

count = 1
for image in images:
    #print(image)
    image_path = os.path.join(dir_path, images[count])
    frame = cv2.imread(image_path)

    out.write(frame) # Write out frame to video
    count+=1
   #cv2.imshow('video',frame)
    if (cv2.waitKey(1) & 0xFF) == ord('q'): # Hit `q` to exit
        break
        
# Release everything if job is finished
out.release()
cv2.destroyAllWindows()

print("The output video is {}".format(output))