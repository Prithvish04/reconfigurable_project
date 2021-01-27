/* Streamulator test platform
 * Original by Michiel van der Vlag, adapted by Matti Dreef
 */

#ifndef INC_H
#define INC_H


#include <stdint.h>
#include <iostream>
#include <hls_stream.h>
#include <hls_video.h>
#include <hls_opencv.h>
#include <ap_axi_sdata.h>

// Image dimensions
#define WIDTH 1280
#define HEIGHT 720

// Number of frames for multi-frame processing
#define FRAMES 1

// Pixel and stream types
typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;


// Stream processing function



//void rgb2gray(pixel_stream &src, pixel_stream &dst);


// Image paths
#define INPUT_IMG  "/home/prithvish/reconfigurable/examples/parrot.jpg"
#define OUTPUT_IMG "/home/prithvish/reconfigurable/examples/out.png"
#define RAW_OUTPUT_IMG "/home/prithvish/reconfigurable/examples/raw.png"


#endif // INC_H
