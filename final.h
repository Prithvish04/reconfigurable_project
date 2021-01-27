#include <stdint.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>


typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;

typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;
typedef hls::LineBuffer<5,WIDTH,pixel_data> linebuffer;


void greyscale(pixel_stream &src, pixel_stream &dst);

void gauss(pixel_stream &src, pixel_stream &dst);

float convolute(int k[3][3], linebuffer& buffer, int x, int y);

void suppression(float angle, pixel_stream &src, pixel_stream &dst);

float sobel_filter(pixel_stream &src, pixel_stream &dst);

void threshold(pixel_stream &src, pixel_stream &dst);

void hystersis(pixel_stream &src, pixel_stream &dst);

