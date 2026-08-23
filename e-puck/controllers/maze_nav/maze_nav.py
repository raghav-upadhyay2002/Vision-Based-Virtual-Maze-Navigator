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
    lower_red1= np.array([7,255,255])
    upper_red2= np.array([170,120,70])
    lower_red2= np.array([180,255,255])

    # inRange() gives a binary mask of pixels inside each range; combining
    # them covers both halves of the red hue band.
    mask1=cv2.inRange(hsv, upper_red1, lower_red1)
    mask2=cv2.inRange(hsv, upper_red2, lower_red2)
    mask= mask1 + mask2

    # Number of red pixels found; used as a simple "is the target visible" threshold.
    pixel_count= cv2.countNonZero(mask)

    if pixel_count> 50:
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

def detect_edges(img_bgr):
    # Convert the image to grayscale
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # Smooth out noise first so Canny doesn't pick up false edges from grain/texture.
    blurred= cv2.GaussianBlur(gray, (5, 5), 0)
    # Canny returns a binary image: white pixels mark detected edges.
    edges= cv2.Canny(blurred, 50, 150)
    return edges

def detect_lines(edges):
    # Probabilistic Hough transform: turns clusters of edge pixels into
    # actual line segments (endpoints), which is more useful than raw edges
    # for recognizing straight maze walls. Currently computed but unused
    # by get_wall_status_vision, which relies on edge density instead.
    lines= cv2.HoughLinesP(
        edges,1,np.pi/180,
        threshold=20,
        minLineLength=15,
        maxLineGap=5
    )
    return lines


def get_wall_status_vision(img_bgr):

    global center_density_count
    global left_density_count
    global right_density_count
    edges= detect_edges(img_bgr)
    lines= detect_lines(edges)
    height, width= edges.shape

    # Split the frame into left/center/right thirds so wall presence can be
    # judged separately in each direction the robot could turn toward.
    left_zone= edges[:,0:int(width*0.33)]
    center_zone= edges[:,int(width*0.33):int(width*0.66)]
    right_zone= edges[:,int(width*0.66):]

    # Fraction of edge pixels in each zone: a wall/obstacle produces a lot of
    # edges, while open space (floor, distant background) produces few.
    left_density= cv2.countNonZero(left_zone) / left_zone.size
    center_density= cv2.countNonZero(center_zone) / center_zone.size
    right_density= cv2.countNonZero(right_zone) / right_zone.size

    # Empirical cutoff: below this edge density, a zone is considered "wall very close"
    # (close enough that it fills the zone as a flat, low-texture surface with few edges).
    density_epsilon= 0.005


    if left_density< density_epsilon and center_density< density_epsilon and right_density< density_epsilon:
        center_density_count+=1

    else:
        center_density_count=0

    wall_ahead= center_density_count>=3  


    if left_density< density_epsilon and right_density> density_epsilon:
        left_density_count+=1

    else: 
        left_density_count=0

    wall_left= left_density_count>=3


    if right_density< density_epsilon and left_density> density_epsilon:
        right_density_count+=1

    else:
        right_density_count=0
    wall_right= right_density_count>=3

    return {
        'wall_ahead': wall_ahead,
        'wall_left': wall_left,
        'wall_right': wall_right,
        'left_density': left_density,
        'center_density': center_density,
        'right_density': right_density
    , 'edges': edges}



    






def get_sensor_debug(distance_sensors):
    # Read the current value from each of the e-puck's infrared proximity
    # sensors (ps0-ps7), for cross-checking the vision-based wall estimate.
    values=[s.getValue() for s in distance_sensors]
    return values



# Webots hands us a Robot instance representing this e-puck; all devices
# (camera, motors, sensors) are accessed through it via getDevice().
robot= Robot()

# Module-level (not local to get_wall_status_vision) so it persists across control-loop
# iterations, counting consecutive frames where all three zones read "wall very close".
center_density_count=0
left_density_count=0
right_density_count=0

# Simulation step size in ms. Every device must be enabled with this value,
# and robot.step(timestep) must be called once per control loop iteration
# to advance the simulation clock.
timestep= int(robot.getBasicTimeStep())


# enable() starts the camera streaming images; without it getImage() returns None.
camera=  robot.getDevice('camera')
camera.enable(timestep)



# The e-puck has 8 built-in infrared proximity sensors (ps0-ps7) ringing its
# body; enabling and collecting them all gives a physical backup for the
# camera-based wall detection above.
ps_names= ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
distance_sensors= []
for name in ps_names:
    sensor= robot.getDevice(name)
    sensor.enable(timestep)
    distance_sensors.append(sensor)







# Setting position to infinity puts the motor in velocity-control mode
# (drive forever) instead of position-control mode (move to an angle then stop).
left_motor= robot.getDevice("left wheel motor")
left_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)





right_motor= robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)



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
   # cv2.imwrite("debug_frame.png", img_bgr)

    # Vision-based wall/opening estimate plus raw IR sensor readings, printed
    # side by side so the two can be compared/debugged against each other.
    wall_status= get_wall_status_vision(img_bgr)
    sensor_values= get_sensor_debug(distance_sensors)

    # Print every frame's booleans unconditionally (not just the one that ends up
    # driving the motors) so wall_left/wall_right are visible even while wall_ahead
    # is the active branch below.
    print("vision-> wall_ahead:", wall_status['wall_ahead'],
          "wall_left:", wall_status['wall_left'],
          "wall_right:", wall_status['wall_right'], end=' ')

    # Check whether the red target is in view and which way it's offset.
    target_visible, target_direction, mask= detect_target(img_bgr)

    if wall_status['wall_ahead']:

            left_motor.setVelocity(-3.0)
            right_motor.setVelocity(3.0)


    elif wall_status['wall_left']:
          
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(1.0)

    elif wall_status['wall_right']:
          
            left_motor.setVelocity(1.0)
            right_motor.setVelocity(3.0)

    elif target_visible:
         if target_direction== 'left':
            left_motor.setVelocity(1.0)
            right_motor.setVelocity(3.0)
         elif target_direction== 'right':
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(1.0)

    else:
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(3.0)

    print("(densities L/C/R:", round(wall_status['left_density'],4),
            round(wall_status['center_density'],4),
            round(wall_status['right_density'],4),")")


    #print("sensors->", sensor_values)

    # Visualize the Canny edge map used for the wall density calculation.
    cv2.imshow("Edges", wall_status['edges'])

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

# Close the preview window once the control loop ends.
cv2.destroyAllWindows()


