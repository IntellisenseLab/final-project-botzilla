import cv2
import freenect
import KobukiDriver as kobuki

my_kobuki = kobuki.Kobuki()

# Function to get RGB and depth data from the Kinect
def get_depth_and_rgb():
    depth, timestamp = freenect.sync_get_depth()
    rgb, timestamp = freenect.sync_get_video()
    return depth, rgb

while True:
    my_kobuki.mov(-100, 100, 0)
    depth, rgb = get_depth_and_rgb()

    # Convert depth data to a grayscale image
    depth = (depth.astype('uint8') >> 2)  # Right shift to make 10-bit data 8-bit
    depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    h, w,_ = rgb.shape
    rgb = rgb[0:h, 50:w-50]

    # Display the RGB and depth streams
    cv2.imshow('RGB', rgb)
    cv2.imshow('Depth', depth)

    # cv2.imwrite('new.jpg', rgb)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()