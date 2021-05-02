#include <stdint.h>
#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_video.h>
#include <math.h>
#include <ap_fixed.h>

#define WIDTH  1920
#define HEIGHT 1080 
//#define WIDTH  1280 for simulation
//#define HEIGHT 720 for simulation
#define HIGH 80
#define LOW 20
#define WEAK 75
#define STRONG 255
#define CORDIC_ITERATIONS 10

// Authors: Group 3
// Course: Reconfigurable Computing

typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;
typedef uint8_t linebuffer2[2][WIDTH];
typedef uint8_t linebuffer4[4][WIDTH];
typedef int16_t linebuffer[2][WIDTH];
typedef uint8_t windowbuffer3[3][3];
typedef uint8_t windowbuffer5[5][5];
typedef ap_uint<1> data_bool;

const int8_t kxy[6][3] = {{-1,0,1},{-2,0,2},{-1,0,1},{1,2,1},{0,0,0},{-1,-2,-1}};
const uint8_t gauss_kernel[25]={1,4,7,4,1,4,16,26,16,4,7,26,41,26,7,4,16,26,16,4,1,4,7,4,1};
const uint8_t angle_step[10]={45,27,14,7,3,2,1,0,0,0};

inline data_bool bound(int16_t row, int16_t col, int8_t i, int8_t j){
	return (row+i)>=0 && (row+i)<HEIGHT && (col+j)>=0 && (col+j)<WIDTH;
}

inline void set_pixel(pixel_data& p, uint8_t intensity){
	p.data = (p.data & 0xFF000000) |(intensity << 16) | (intensity << 8) | intensity ;
}

inline uint8_t get_value(pixel_data& p){
	return p.data & 0x000000FF;
}

inline void read_pixel(pixel_stream &src, pixel_data& p, uint16_t& x, uint16_t& y){
	src >> p;
	if (p.user)
		x = y = 0;
}

inline void write_pixel(pixel_stream &dst, pixel_data& p, uint16_t& x, uint16_t& y){
	if (p.last){
		x = 0;
		y++;
	}
	else
		x++;
	dst << p;
}

inline void update5(linebuffer4& buffer, windowbuffer5& window, pixel_data& p, uint16_t x){

	uint8_t tmp[5];
	tmp[0] = buffer[0][x];
	for (uint8_t i = 1; i < 4; i++){
		buffer[i-1][x] = buffer[i][x];
		tmp[i] = buffer[i-1][x];
	}

	buffer[3][x] = get_value(p);
	tmp[4] = buffer[3][x];

	for (uint8_t i= 0; i < 5; i++)
		for (uint8_t j = 0; j < 4; j++)
			window[i][j] = window[i][j + 1];

	for (uint8_t i = 0; i < 5; i++)
		window[i][4] = tmp[i];
}

inline void update3(linebuffer2& buffer, windowbuffer3& window, pixel_data& p, uint16_t x){

	uint8_t tmp[3];
	tmp[0] = buffer[0][x];

	buffer[0][x] = buffer[1][x];
	tmp[1] = buffer[0][x];

	buffer[1][x] = get_value(p);
	tmp[2] = buffer[1][x];

	for (uint8_t i= 0; i < 3; i++)
		for (uint8_t j = 0; j < 2; j++)
			window[i][j] = window[i][j + 1];

	for (uint8_t i = 0; i < 3; i++)
		window[i][2] = tmp[i];
}

inline void update_angle(int16_t angle, linebuffer& angle_buff, uint16_t x){
	angle_buff[0][x] = angle_buff[1][x];
	angle_buff[1][x] = angle;
}

inline int16_t convolute(windowbuffer3& window, uint16_t x, uint16_t y, data_bool mask){

	int16_t result = 0;
	int16_t cy=y-1, cx=x-1;
	for(int8_t i=-1; i<2; i++){
		for(int8_t j=-1; j<2; j++){
			if(bound(cy, cx, i, j))
				result += (int16_t) window[i+1][j+1] * kxy[i+1+(mask?3:0)][j+1];
		}
	}
	return result;
}

inline int16_t sobel_v1(int16_t i_x, int16_t i_y, pixel_data& p){

	int32_t sqr1 = i_x * i_x;
	int32_t sqr2 = i_y * i_y;
	int32_t sum = sqr1 + sqr2;
	uint8_t intensity = (uint8_t) hls::sqrt(sum);
	set_pixel(p, intensity);

	return (int16_t) (hls::atan2(i_y, i_x) * 180 / M_PI);
}

inline int16_t sobel_v2(int16_t i_x, int16_t i_y, pixel_data& p){

	int16_t atan=0;
	int16_t x_cordic[CORDIC_ITERATIONS],y_cordic[CORDIC_ITERATIONS];
	data_bool sigma;
	data_bool x_sig,y_sig;

	x_cordic[0]=i_x;
	y_cordic[0]=i_y;

	for (uint8_t j = 1; j < CORDIC_ITERATIONS; j++){
		x_sig=(x_cordic[j-1]>=0)?1:0;
		y_sig=(y_cordic[j-1]>=0)?1:0;
		sigma= (x_sig==y_sig)?0:1;
		x_cordic[j] = (sigma) ? x_cordic[j-1] - (y_cordic[j-1]>> (j - 1)) : x_cordic[j-1] + (y_cordic[j-1] >> (j - 1));
		y_cordic[j] = (sigma) ? y_cordic[j-1] + (x_cordic[j-1] >> (j - 1)) : y_cordic[j-1] - (x_cordic[j-1] >> (j - 1));
		atan = (sigma) ? atan - angle_step[j-1 ] : atan + angle_step[j-1];
	}

	uint16_t intensity_interim=(x_cordic[CORDIC_ITERATIONS-1]>=0)?x_cordic[CORDIC_ITERATIONS-1]:-x_cordic[CORDIC_ITERATIONS-1];

	//multiply with a constant 0.6094
	uint8_t intensity = (uint8_t) ((intensity_interim>>1)+(intensity_interim>>2)-(intensity_interim>>3)-
			(intensity_interim>>4)+(intensity_interim>>5)+(intensity_interim>>6));

	set_pixel(p, intensity);

	return atan;
}

void greyscale(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;
	pixel_data p;

	read_pixel(src, p, x, y);

	uint8_t r,g,b,a;
	r= (uint8_t) (p.data)&0x000000FF;
	g= (uint8_t) (p.data>>8)&0x000000FF;
	b= (uint8_t) (p.data>>16)&0x000000FF;
	a= (uint8_t) (p.data>>24)&0x000000FF;
	uint8_t intensity = (r>>2) + (r>>5) + (b>>4) + (b>>5)+ (g>>1) + (g>>4);

	set_pixel(p, intensity);

	write_pixel(dst, p, x, y);
}

void gauss(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;
	static linebuffer4 buffer;
	static windowbuffer5 window;
	pixel_data p;

	read_pixel(src, p, x, y);

#pragma HLS ARRAY_PARTITION variable=buffer complete dim=1
#pragma HLS ARRAY_PARTITION variable=window complete dim=0
#pragma HLS dependence variable=buffer inter false

	update5(buffer, window, p, x);

	if(x>1 && y>1){
		int32_t result = 0;
		int16_t cy=y-2, cx=x-2;
		for(int8_t i=-2; i<3; i++){
			for(int8_t j=-2; j<3; j++){
				if(bound(cy, cx, i, j))
					result += window[i+2][j+2] * gauss_kernel[(5*(j+2))+i+2];
			}
		}
		result /= 273;
		set_pixel(p, (uint8_t) result);
	}
	write_pixel(dst, p, x, y);
}


int16_t sobel(pixel_stream &src, pixel_stream &dst, uint32_t mask){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS INTERFACE s_axilite port=mask
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;
	static linebuffer2 buffer;
	static windowbuffer3 window;
	int16_t angle = 0;
	pixel_data p;

	read_pixel(src, p, x, y);

#pragma HLS ARRAY_PARTITION variable=buffer complete dim=1
#pragma HLS ARRAY_PARTITION variable=window complete dim=0
#pragma HLS dependence variable=buffer inter false

	if(x>1 && y>1)
		update3(buffer, window, p, x);

	if(y>2 && x>2){
		int16_t i_x = convolute(window, x, y, 0);
		int16_t i_y = convolute(window, x, y, 1);

		if(mask == 0)
			angle = sobel_v1(i_x, i_y, p);
		else
			angle = sobel_v2(i_x, i_y, p);
	}

	write_pixel(dst, p, x, y);

	return angle;
}

void suppression(pixel_stream &src, pixel_stream &dst, int16_t& p_angle){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE ap_none port=&p_angle
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;
	static linebuffer2 buffer;
	static windowbuffer3 window;
	static linebuffer angle_buff;
	pixel_data p;

	read_pixel(src, p, x, y);

#pragma HLS ARRAY_PARTITION variable=buffer complete dim=1
#pragma HLS ARRAY_PARTITION variable=angle_buff complete dim=1
#pragma HLS ARRAY_PARTITION variable=window complete dim=0
#pragma HLS dependence variable=buffer inter false

	if(x>2 && y>2){
		update3(buffer, window, p, x);
		update_angle(p_angle, angle_buff, x);
	}

	int16_t cy=y-1, cx=x-1;
	uint8_t q=255, r=255;
	int16_t angle = 0;

	if(y>3 && x>3){
		angle = angle_buff[0][cx];
		if(angle < 0)
			angle += 180;

		if((0 <= angle < 22.5) or (157.5 <= angle <= 180)){
			if(bound(cy, cx, 0, 1))
				q = window[1][2];
			if(bound(cy, cx, 0, -1))
				r = window[1][0];
		}else if(22.5 <= angle < 67.5){
			if(bound(cy, cx, 1, -1))
				q = window[2][0];
			if(bound(cy, cx, -1, 1))
				r = window[0][2];
		}else if(67.5 <= angle < 112.5){
			if(bound(cy, cx, 1, 0))
				q = window[2][1];
			if(bound(cy, cx, -1, 0))
				r = window[0][1];
		}else if(112.5 <= angle < 157.5){
			if(bound(cy, cx, -1, -1))
				q = window[0][0];
			if(bound(cy, cx, 1, 1))
				r = window[2][2];
		}

		uint8_t value = window[1][1];
		if(value >= q && value >=r )
			set_pixel(p, value);
		else
			set_pixel(p, 0);
	}

	write_pixel(dst, p, x, y);
}

void threshold(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

    static uint16_t x = 0;
    static uint16_t y = 0;
	pixel_data p;

	read_pixel(src, p, x, y);

    uint8_t data;

    if(y>3 && x>3){
    	data = get_value(p);
		if(data>= HIGH)
			set_pixel(p, STRONG);
		else if(data>= LOW)
			set_pixel(p, WEAK);
		else
			set_pixel(p, 0);
    }

	write_pixel(dst, p, x, y);
}

void hysteresis(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
    static uint16_t y = 0;
	static linebuffer2 buffer;
	static windowbuffer3 window;
	pixel_data p;

	read_pixel(src, p, x, y);

#pragma HLS ARRAY_PARTITION variable=buffer complete dim=1
#pragma HLS ARRAY_PARTITION variable=window complete dim=0
#pragma HLS dependence variable=buffer inter false

	if(x>3 && y>3)
		update3(buffer, window, p, x);

	uint8_t data;
	data_bool flag;
	int16_t cy=y-1, cx=x-1;

    if(y>5 && x>5){
    	data = window[1][1];
		flag = 0;
		if(data == WEAK){
			for(int8_t i =-1; i < 2; i++){
				for(int8_t j=-1; j< 2; j++){
					if(bound(cy,cx,i,j)){
						if(window[i+1][j+1] == STRONG){
							flag = 1;
						}
					}
				}
				if(flag)
					break;
			}

			if(flag){
				set_pixel(p, STRONG);
				window[1][1] = STRONG;
			}else{
				set_pixel(p, 0);
				window[1][1] = 0;
			}
		}else
			set_pixel(p, data);

    }else if(y>4 && x>4)
    	set_pixel(p, data);
    else
    	set_pixel(p, 0);

	write_pixel(dst, p, x, y);
}
