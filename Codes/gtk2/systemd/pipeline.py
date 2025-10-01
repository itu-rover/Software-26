
def pipeline(device, input_type, width, height,encoder, 
             bitrate, host, port, flip):
    
    pipeline = "gst-launch-1.0 "
    
    # io-mode for mpp encoders
    if "mpp" in encoder:
        pipeline += f"v4l2src device={device} io-mode=4 ! "
    else:
        pipeline += f"v4l2src device={device} ! "
        
    #
    if "MJPEG" in input_type:
        pipeline += f"image/jpeg,width={width},height={height} ! jpegdec ! "
    elif "YUY" in input_type:
        pipeline += f"video/x-raw,width={width},height={height} !"
        
    for flip_direction in flip:
        pipeline += f"videoflip video-direction={flip_direction} ! "
    
    pipeline += "videoconvert ! video/x-raw,format=NV12 ! "
    
    pipeline += f"{encoder} rc-mode=cbr bps={bitrate} gop=15 ! "
    
    if "265" in encoder:
        pipeline += f"h265parse ! rtph265pay config-interval=1 pt=96 ! udpsink host={host} port={port} sync=false &"
    elif "264" in encoder:
        pipeline += f"h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host={host} port={port} sync=false &"
    
    return pipeline

device = "/dev/video0"
input_type = "MJPEG"
width = 640
height = 480
encoder = "mpph265enc"
bitrate = 100000
host = "192.168.1.6"
port = 5000
flip = ["horiz", "vert"]


print(pipeline(device, input_type, width, height, encoder, bitrate, host, port, flip))

