from controller import Robot
import numpy as np
import cv2


def detect_target(img_bgr):

    hsv= cv2.cvtColor(img_bgr, cv2.color_BGR2HSV)


    upper_red1= np.array([0,120,70])
    lower_red1= np.array([10,255,255])
    upper_red2= np.array([170,120,70]) 
    lower_red2= np.array([180,255,255])

    mask1=cv2.inRange(hsv, upper_red1, lower_red1)
    mask2=cv2.inRange(hsv, upper_red2, lower_red2)
    mask= mask1 + mask2

    pixel_count= cv2.countNonZero(mask)

    if pixel_count> 20:
        ys, xs= np.where(mask>0)
        
        




# Webots hands us a Robot instance representing this e-puck; all devices
# (camera, motors, sensors) are accessed through it via getDevice().
robot= Robot()



# Simulation step size in ms. Every device must be enabled with this value,
# and robot.step(timestep) must be called once per control loop iteration
# to advance the simulation clock.
timestep= int(robot.getBasicTimeStep())


# enable() starts the camera streaming images; without it getImage() returns None.
camera=  robot.getDevice('camera')
camera.enable(timestep)


# Setting position to infinity puts the motor in velocity-control mode
# (drive forever) instead of position-control mode (move to an angle then stop).
left_motor= robot.getDevice("left wheel motor")
left_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)





right_motor= robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)



keyboard= robot.getKeyboard()
keyboard.enable(timestep)

print('Camera resolution:', camera.getWidth() ,'x' , camera.getHeight())


while robot.step(timestep)!=-1:
    # Raw camera image comes back as a flat byte buffer in BGRA order.
    image= camera.getImage()
    width= camera.getWidth()
    height=camera.getHeight()

    # Reshape the flat buffer into a (height, width, 4) BGRA array OpenCV can use.
    img= np.frombuffer(image, np.uint8).reshape((height,width,4))

    # Drop the alpha channel so it's a standard 3-channel BGR image.
    img_bgr= cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)


    # Show a resized preview of what the robot's camera currently sees.
    cv2.imshow("Robot Camera Feed", cv2.resize(img_bgr, (300, 300)))

    # Required for OpenCV to actually paint/refresh the window each frame.
    cv2.waitKey(1)


    key= keyboard.getKey()
    speed= 4.0

    if key== ord('W'):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)

    elif key== ord('S'):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(-speed)

    elif key== ord('A'):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)

    elif key== ord('D'):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)

    else:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

# Close the preview window once the control loop ends.
cv2.destroyAllWindows()


