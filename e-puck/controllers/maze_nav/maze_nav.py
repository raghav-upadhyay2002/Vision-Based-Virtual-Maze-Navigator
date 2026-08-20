from controller import Robot
import numpy as np
import cv2


def detect_target(img_bgr):
    # HSV separates color (hue) from lighting (value), which makes
    # thresholding a specific color far more reliable than in raw BGR.
    hsv= cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Red wraps around the hue wheel (near 0 and near 180), so two ranges
    # are needed to catch both ends and cover the full red band.
    upper_red1= np.array([0,120,70])
    lower_red1= np.array([10,255,255])
    upper_red2= np.array([170,120,70])
    lower_red2= np.array([180,255,255])

    # inRange() gives a binary mask of pixels inside each range; combining
    # them covers both halves of the red hue band.
    mask1=cv2.inRange(hsv, upper_red1, lower_red1)
    mask2=cv2.inRange(hsv, upper_red2, lower_red2)
    mask= mask1 + mask2

    # Number of red pixels found; used as a simple "is the target visible" threshold.
    pixel_count= cv2.countNonZero(mask)

    if pixel_count> 20:
        # Average x-position of every red pixel gives the target's horizontal
        # center in the frame, which tells us which way to steer.
        ys, xs= np.where(mask>0)
        cx=np.mean(xs)
        width= img_bgr.shape[1]

        if cx<width*0.4:
            direction= 'left'
        elif cx>width*0.6:
            direction= 'right'
        else:
            direction= 'center'

        return True, direction , mask
    else:
        return False, None, mask

        
        




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



# Lets the simulation window capture WASD key presses for manual driving.
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


    # Check whether the red target is in view and which way it's offset.
    target_visible, target_direction,mask= detect_target(img_bgr)

    if target_visible:
        print('Target detected! Direction:', target_direction)
        # Visualize exactly which pixels were classified as "red target".
        cv2.imshow("Target Mask", mask)


    # Show a resized preview of what the robot's camera currently sees.
    cv2.imshow("Robot Camera Feed", cv2.resize(img_bgr, (300, 300)))

    # Required for OpenCV to actually paint/refresh the window each frame.
    cv2.waitKey(1)


    # getKey() returns the currently pressed key each step (or -1 if none),
    # so this re-evaluates drive direction every frame — no key means stop.
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


