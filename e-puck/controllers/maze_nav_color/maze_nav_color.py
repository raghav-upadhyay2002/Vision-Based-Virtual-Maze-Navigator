from controller import Robot
import cv2
import numpy as np

robot= Robot()
time_step= int(robot.getBasicTimeStep())

camera= robot.getDevice('camera')
camera.enable(time_step)

left_motor= robot.getDevice('left wheel motor')
right_motor= robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

print('Camera resolution:', camera.getWidth() ,'x' , camera.getHeight())


robot.step(time_step)
width= camera.getWidth()
height= camera.getHeight()


def detect_target(img_bgr):

    hsv= cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    upper_red1= np.array([10, 255, 255])
    lower_red1= np.array([0, 100, 100])

    upper_red2= np.array([180, 255, 255])
    lower_red2= np.array([160, 100, 100])

    mask1= cv2.inRange(hsv, lower_red1, upper_red1)
    mask2= cv2.inRange(hsv, lower_red2, upper_red2)
    mask=mask1+mask2


    pixel_count= cv2.countNonZero(mask)

    if pixel_count> 50:
        ys,xs= np.where(mask>0)
        x_mean= np.mean(xs)

        width= img_bgr.shape[1]

        if x_mean< width/3:
            return True, 'left', mask
        elif x_mean> 2*width/3:
            return True, 'right', mask
        else:
            return True, 'center', mask

    else:
        return False, None, mask



def bottom_img(img_bgr, split_ratio):
    height, width= img_bgr.shape[:2]

    split_y= int(height*(1-split_ratio))
    img_bottom= img_bgr[split_y:,:]


    return img_bottom

def detect_edges(img_bottom):

    gray_img= cv2.cvtColor(img_bottom, cv2.COLOR_BGR2GRAY)
    blurred= cv2.GaussianBlur(gray_img, (5,5),0)
    edges= cv2.Canny(blurred,50,150)
    return edges


def detect_line(edges):
    lines= cv2.HoughLinesP(
        edges,1,np.pi/180,
        threshold=20,
        minLineLength=15,
        maxLineGap=5)
    return lines 



def detect_walls_status(img_bgr, split_ratio):

    img_bottom= bottom_img(img_bgr, split_ratio)

    edges= detect_edges(img_bottom)

    lines= detect_line(edges)

    height, width= edges.shape

    left_zone= edges[:,0:int(width*0.33)]
    center_zone= edges[:,int(width*0.33):int(width*0.66)]
    right_zone= edges[:,int(width*0.66):]

    left_density= cv2.countNonZero(left_zone) / left_zone.size
    center_density= cv2.countNonZero(center_zone) / center_zone.size
    right_density= cv2.countNonZero(right_zone) / right_zone.size

    max_thresold= 0.02

    wall_left= left_density> max_thresold
    wall_ahead= center_density> max_thresold
    wall_right= right_density> max_thresold

    return {
        'wall_ahead': wall_ahead,
        'wall_left': wall_left,
        'wall_right': wall_right,
        'left_density': left_density,
        'center_density': center_density,
        'right_density': right_density,
        'edges': edges
    }











        



while robot.step(time_step)!=-1:
    # Raw camera image comes back as a flat byte buffer in BGRA order.
    image= camera.getImage()
    width= camera.getWidth()
    height=camera.getHeight()
    img= np.frombuffer(image, np.uint8).reshape((height,width,4))

    img_bgr= cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    wall_status= detect_walls_status(img_bgr, 0.2)
    print("vision-> wall_ahead:", wall_status['wall_ahead'],
          "wall_left:", wall_status['wall_left'],
          "wall_right:", wall_status['wall_right'], end=' ')

    print("(densities L/C/R:", round(wall_status['left_density'],4),
          round(wall_status['center_density'],4),
          round(wall_status['right_density'],4),")")

    left_motor.setVelocity(1.0)
    right_motor.setVelocity(1.0)
    cv2.imshow("Edges", wall_status['edges'])
    cv2.imshow("Camera", img_bgr)
    cv2.waitKey(1)

cv2.destroyAllWindows()