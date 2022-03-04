# coding=UTF-8
import numpy as np
import cv2

def load_radar(example_path):# AnyStr) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Decode a single Oxford Radar RobotCar Dataset radar example
    Args:
        example_path (AnyStr): Oxford Radar RobotCar Dataset Example png
    Returns:
        timestamps (np.ndarray): Timestamp for each azimuth in int64 (UNIX time)
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        valid (np.ndarray) Mask of whether azimuth data is an original sensor reading or interpolated from adjacent
            azimuths
        fft_data (np.ndarray): Radar power readings along each azimuth
        radar_resolution (float): Resolution of the polar radar data (metres per pixel)
    """
    # 硬编码配置，以简化解析代码
    radar_resolution = np.array([0.0432], np.float32)
    encoder_size = 5600 # 编码尺寸：azimuth (radians) = ( sweep_counter / encoder_size) * 2pi
    # UNIX时间戳记在第1-8列中为int64
    # 第9-10列中为uint16的sweep_counter，提取并转换为转换为角度
    # 第11列中的有效标志为uint8
    # 11列往后均为float32的雷达fft数据（功率*255），颜色越白，功率越大（功率(像素值)比值位于(0,1)之间）
    raw_example_data = cv2.imread(example_path, cv2.IMREAD_GRAYSCALE)
    timestamps = raw_example_data[:, :8].copy().view(np.int64)
    azimuths = (raw_example_data[:, 8:10].copy().view(np.uint16) / float(encoder_size) * 2 * np.pi).astype(np.float32)
    valid = raw_example_data[:, 10:11] == 255
    fft_data = raw_example_data[:, 11:].astype(np.float32)[:, :, np.newaxis] / 255. # 获取的数值再除以255（白色像素值）

    return timestamps, azimuths, valid, fft_data, radar_resolution

def radar_polar_to_cartesian(azimuths, fft_data, radar_resolution,
                             cart_resolution, cart_pixel_width, interpolate_crossover=True):
    """Convert a polar radar scan to cartesian.
    Args:
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        fft_data (np.ndarray): Polar radar power readings
        radar_resolution (float): Resolution of the polar radar data (metres per pixel)
        cart_resolution (float): Cartesian resolution (metres per pixel)
        cart_pixel_size (int): Width and height of the returned square cartesian output (pixels). Please see the Notes
            below for a full explanation of how this is used.
        interpolate_crossover (bool, optional): If true interpolates between the end and start  azimuth of the scan. In
            practice a scan before / after should be used but this prevents nan regions in the return cartesian form.

    Returns:
        np.ndarray: Cartesian radar power readings
    Notes:
        After using the warping grid the output radar cartesian is defined as as follows where
        X and Y are the `real` world locations of the pixels in metres:
         If 'cart_pixel_width' is odd:
                        +------ Y = -1 * cart_resolution (m)
                        |+----- Y =  0 (m) at centre pixel
                        ||+---- Y =  1 * cart_resolution (m)
                        |||+--- Y =  2 * cart_resolution (m)
                        |||| +- Y =  cart_pixel_width // 2 * cart_resolution (m) (at last pixel)
                        |||| +-----------+
                        vvvv             v
         +---------------+---------------+
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         +---------------+---------------+ <-- X = 0 (m) at centre pixel
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         |               |               |
         +---------------+---------------+
         <------------------------------->
             cart_pixel_width (pixels)
         If 'cart_pixel_width' is even:
                        +------ Y = -0.5 * cart_resolution (m)
                        |+----- Y =  0.5 * cart_resolution (m)
                        ||+---- Y =  1.5 * cart_resolution (m)
                        |||+--- Y =  2.5 * cart_resolution (m)
                        |||| +- Y =  (cart_pixel_width / 2 - 0.5) * cart_resolution (m) (at last pixel)
                        |||| +----------+
                        vvvv            v
         +------------------------------+
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         |                              |
         +------------------------------+
         <------------------------------>
             cart_pixel_width (pixels)
    """
    if (cart_pixel_width % 2) == 0:
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution
    else:
        cart_min_range = cart_pixel_width // 2 * cart_resolution
    coords = np.linspace(-cart_min_range, cart_min_range, cart_pixel_width, dtype=np.float32)
    # 生成网格点坐标矩阵
    Y, X = np.meshgrid(coords, -coords)
    sample_range = np.sqrt(Y * Y + X * X)
    sample_angle = np.arctan2(Y, X)
    sample_angle += (sample_angle < 0).astype(np.float32) * 2. * np.pi

    # Interpolate Radar Data Coordinates，(u,v)像素点插值矩阵，u,v分别为向量
    azimuth_step = azimuths[1] - azimuths[0]
    sample_u = (sample_range - radar_resolution / 2) / radar_resolution
    sample_v = (sample_angle - azimuths[0]) / azimuth_step

    # We clip the sample points to the minimum sensor reading range so that we
    # do not have undefined results in the centre of the image. In practice
    # this region is simply undefined.
    sample_u[sample_u < 0] = 0  # 横坐标小于0者，归0：说明sample_range - radar_resolution / 2 小于0

    if interpolate_crossover:   # 对图像fft_data进行双边插值
        fft_data = np.concatenate((fft_data[-1:], fft_data, fft_data[:1]), 0)# axis = 0
        sample_v = sample_v + 1
    # numpy.stack(arrays, axis=0)：沿着新轴连接数组的序列。
    # axis=0，第一个维度，axis=-1，最后一个维度。
    polar_to_cart_warp = np.stack((sample_u, sample_v), -1)
    # cv2.remap：把一幅图像内的像素点放置到另外一幅图像内的指定位置，这个过程称为重映射.
    cart_img = np.expand_dims(cv2.remap(fft_data, polar_to_cart_warp, None, cv2.INTER_LINEAR), -1)
    return cart_img

    
