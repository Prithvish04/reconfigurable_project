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
void greyscale( pixel_stream&,  pixel_stream&);
void gauss(pixel_stream&,  pixel_stream&);
void convolute_x(pixel_stream&,  pixel_stream&, int32_t&);
void convolute_y(pixel_stream&,  pixel_stream&, int32_t&);
void sobel_filter(pixel_stream&, pixel_stream&, int32_t&, int32_t&, float&);
void suppression(float&, pixel_stream&,  pixel_stream&);
void threshold(pixel_stream &, pixel_stream &);
void hystersis(pixel_stream &, pixel_stream &);
void correction(pixel_stream &, pixel_stream &);

// Image paths
#define INPUT_IMG  "C:/Users/HashPac/Desktop/School/Master/Q2/Reconfigurable_Computing/Lab/PYNQ-Z2_lab2020/examples/parrot.jpg"
#define OUTPUT_IMG "C:/Users/HashPac/Desktop/School/Master/Q2/Reconfigurable_Computing/Lab/PYNQ-Z2_lab2020/examples/output.png"
#define RAW_OUTPUT_IMG "C:/Users/HashPac/Desktop/School/Master/Q2/Reconfigurable_Computing/Lab/PYNQ-Z2_lab2020/examples/raw_output.png"


#endif // INC_H
