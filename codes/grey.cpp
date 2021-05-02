#include <stdint.h>
#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_video.h>
#include <math.h>
#include <ap_fixed.h>

#define WIDTH 1280
#define HEIGHT 720

typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;
typedef uint8_t linebuffer3[3][WIDTH];
typedef float floatbuffer3[3][WIDTH];
typedef uint8_t linebuffer5[5][WIDTH];
typedef ap_uint<1> data_bool;
const int8_t kxy[6][3] = {{-1,0,1},{-2,0,2},{-1,0,1},{1,2,1},{0,0,0},{-1,-2,-1}};
const uint8_t gauss_kernel[25]={1,4,7,4,1,4,16,26,16,4,7,26,41,26,7,4,16,26,16,4,1,4,7,4,1};

inline bool buffer_condition5(int cy, int cx, int y){
#pragma HLS inline
	return (cy!=1 || y>=3) && (cy!=0 || y>=4) && cx>=0 && cx<WIDTH && cy>=0 && cy<5;
}

inline bool buffer_condition3(int cy, int cx, int y){
#pragma HLS inline
	return (cy!=0 || y>=2) && cx>=0 && cx<WIDTH && cy>=0 && cy<3;
}

inline void set_pixel(pixel_data& p, uint8_t intensity){
#pragma HLS inline
	p.data = (p.data & 0xFF000000) |(intensity << 16) | (intensity << 8) | intensity ;
}

inline uint8_t get_value(pixel_data& p){
#pragma HLS inline
	return p.data & 0x000000FF;
}

void greyscale(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II = 1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	src >> p;
	if (p.user)
		x = y = 0;

	uint8_t r,g,b,a;
	r= (uint8_t) (p.data)&0x000000FF;
	g= (uint8_t) (p.data>>8)&0x000000FF;
	b= (uint8_t) (p.data>>16)&0x000000FF;
	a= (uint8_t) (p.data>>24)&0x000000FF;
	uint8_t intensity = (r>>2) + (r>>5) + (b>>4) + (b>>5)+ (g>>1) + (g>>4);

	set_pixel(p, intensity);

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void gauss(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	pixel_data current;
	int16_t xi = x-2, yi = 2;
	src >> p;
	if (p.user)
		x = y = 0;

	static linebuffer5 buffer;
#pragma HLS ARRAY_PARTITION variable=buffer cyclic factor=2 dim=1
#pragma HLS dependence variable=buffer inter false

	buffer[0][x]=buffer[1][x];
	buffer[1][x]=buffer[2][x];
	buffer[2][x]=buffer[3][x];
	buffer[3][x]=buffer[4][x];
	buffer[4][x]=get_value(p);

	uint8_t value;
	int result = 0;

	if(y>1 && x>1){
		for(int8_t i=-2; i<3; i++){
			for(int8_t j=-2; j<3; j++){
#pragma HLS unroll
				int16_t cx = x - 2 + i;
				int16_t cy = 2 + j;
				if(buffer_condition5(cy,cx,y)){
					value = buffer[cy][cx];
					result += value * gauss_kernel[5*(j+2)+i+2];
				}
			}
		}
		result /= 273;
		set_pixel(p, (uint8_t) result);
	}

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void convolute_x(pixel_stream &src, pixel_stream &dst, int32_t& sum){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS INTERFACE ap_none port=&sum
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	src >> p;
	if (p.user)
		x = y = 0;

	static linebuffer3 buffer;
#pragma HLS ARRAY_PARTITION variable=buffer cyclic factor=2 dim=1
#pragma HLS dependence variable=buffer inter false

	if(y>1 && x>1){
		buffer[0][x]=buffer[1][x];
		buffer[1][x]=buffer[2][x];
		buffer[2][x]=get_value(p);
	}

	sum = 0;
	if(y>2 && x>2){
		data_bool xy = 0;
		uint8_t value;
		for(int8_t i=-1; i<2; i++){
			for(int8_t j=-1; j<2; j++){
#pragma HLS unroll
				int16_t cx = x - 1 + i;
				int16_t cy = 1 + j;
				if(buffer_condition3(cy, cx, y)){
					value = buffer[cy][cx];
					sum += (int32_t) value * kxy[i+1+(xy?3:0)][j+1];
				}
			}
		}
	}

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void convolute_y(pixel_stream &src, pixel_stream &dst, int32_t& sum){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS INTERFACE ap_none port=&sum
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	src >> p;
	if (p.user)
		x = y = 0;

	static linebuffer3 buffer;
#pragma HLS ARRAY_PARTITION variable=buffer cyclic factor=2 dim=1
#pragma HLS dependence variable=buffer inter false

	if(y>1 && x>1){
		buffer[0][x]=buffer[1][x];
		buffer[1][x]=buffer[2][x];
		buffer[2][x]=get_value(p);
	}

	sum = 0;
	if(y>2 && x>2){
		data_bool xy = 1;
		uint8_t value;
		for(int8_t i=-1; i<2; i++){
			for(int8_t j=-1; j<2; j++){
#pragma HLS unroll
				int16_t cx = x - 1 + i;
				int16_t cy = 1 + j;
				if(buffer_condition3(cy, cx, y)){
					value = buffer[cy][cx];
					sum += (int32_t) value * kxy[i+1+(xy?3:0)][j+1];
				}
			}
		}
	}

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void sobel_filter(pixel_stream &src, pixel_stream &dst, int32_t& i_x, int32_t& i_y, float& angle){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS INTERFACE ap_none port=&angle
#pragma HLS INTERFACE ap_none port=&i_x
#pragma HLS INTERFACE ap_none port=&i_y
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive

	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	pixel_data current;
	int16_t xi = x-1, yi = 1;
	src >> p;
	if (p.user)
		x = y = 0;

	angle = 0.0;
	if(y>2 && x>2){
		int32_t sqr1 = i_x * i_x;
		int32_t sqr2 = i_y * i_y;
		int32_t sum = sqr1 + sqr2;
		uint8_t intensity = (uint8_t) hls::sqrt(sum);
		set_pixel(p, intensity);

		angle = hls::atan2(i_y, i_x) * 180 / M_PI;
	}

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void suppression(float& p_angle, pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS INTERFACE ap_none port=&p_angle
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive
	static uint16_t x = 0;
	static uint16_t y = 0;

	pixel_data p;
	uint8_t current;
	int16_t xi = x-1, yi = 1;
	float angle;
	src >> p;
	if (p.user)
		x = y = 0;

	uint8_t result;
	uint8_t q = 255, r = 255;
	int16_t cx, cy;

	static linebuffer3 buffer;
	static floatbuffer3 buffer_angle;
#pragma HLS ARRAY_PARTITION variable=buffer cyclic factor=2 dim=1
#pragma HLS ARRAY_PARTITION variable=buffer_angle cyclic factor=2 dim=1
#pragma HLS dependence variable=buffer inter false

    if(y>2 && x>2){
    	buffer[0][x]=buffer[1][x];
    	buffer[1][x]=buffer[2][x];
		buffer[2][x]=get_value(p);
		buffer_angle[0][x]=buffer_angle[1][x];
		buffer_angle[1][x]=buffer_angle[2][x];
		buffer_angle[2][x]=p_angle;
    }

	if(y>3 && x>3){
    	current = buffer[yi][xi];
		angle = buffer_angle[yi][xi];

		if(angle < 0){
			angle += 180;
		}

		if((0 <= angle < 22.5) or (157.5 <= angle <= 180)){
			cx = xi + 1;
			cy = yi;
			if(buffer_condition3(cy,cx, y)){
				q = buffer[cy][cx];
			}
			cx = xi -1;
			if(buffer_condition3(cy,cx, y)){
				r = buffer[cy][cx];
			}
		}else if(22.5 <= angle < 67.5){
			cx = xi - 1;
			cy = yi + 1;
			if(buffer_condition3(cy,cx, y)){
				q =  buffer[cy][cx];
			}
			cx = xi + 1;
			cy = yi - 1;
			if(buffer_condition3(cy,cx, y)){
				r =  buffer[cy][cx];
			}
		}else if(67.5 <= angle < 112.5){
			cx = xi;
			cy = yi + 1;
			if(buffer_condition3(cy,cx,y)){
				q = buffer[cy][cx];
			}
			cy = yi - 1;
			if(buffer_condition3(cy,cx,y)){
				r = buffer[cy][cx];
			}
		}else if(112.5 <= angle < 157.5){
			cx = xi - 1;
			cy = yi - 1;
			if(buffer_condition3(cy,cx,y)){
				q = buffer[cy][cx];
			}
			cx = xi + 1;
			cy = yi + 1;
			if(buffer_condition3(cy,cx,y)){
				r = buffer[cy][cx];
			}
		}

		if(current >= q && current >=r ){
			set_pixel(p, current);
		}else{
			set_pixel(p, 0);
		}
	}

	if (p.last){
		x = 0;
		y++;
	}
	else{
		x++;
	}

	dst << p;
}

void threshold(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive
	// we assume the highest pixel value is 255
	// the ratio of high threshold is 0.9
	// the ratio of low threshold is 0.5

    static uint16_t x = 0;
    static uint16_t y = 0;

    pixel_data p;
    src >> p;
    if (p.user)
            x = y = 0;

    uint8_t low_threshold = 15;
    uint8_t high_threshold = 50;
    uint8_t weak = 30;
    uint8_t strong = 255;
    uint8_t pixel_data;

    if(y>3 && x>3){
    	pixel_data = get_value(p);
		if( pixel_data >= high_threshold){
			set_pixel(p, strong);
		}
		else if(pixel_data>= low_threshold){
			set_pixel(p, weak);
		}
		else {
			set_pixel(p, 0);
		}
    }

    if (p.last){
    	x = 0;
        y++;
     }
     else{
        x++;
     }

     dst << p;
}


void hystersis(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
#pragma HLS PIPELINE II=1
#pragma HLS inline region recursive
	// we assume the highest pixel value is 255
	// the ratio of high threshold is 0.9
	// the ratio of low threshold is 0.5
    static uint16_t x = 0;
    static uint16_t y = 0;

    pixel_data p;
	pixel_data current;
	int16_t xi = x-1, yi = 1;
    src >> p;
    if (p.user)
            x = y = 0;

    data_bool flag;
    uint8_t value;
    int16_t cy, cx;
    uint8_t weak = 30;
    uint8_t strong = 255;

    static linebuffer3 buffer;
#pragma HLS ARRAY_PARTITION variable=buffer cyclic factor=2 dim=1
#pragma HLS dependence variable=buffer inter false

    if(y>3 && x>3){
    	buffer[0][x]=buffer[1][x];
    	buffer[1][x]=buffer[2][x];
		buffer[2][x]=get_value(p);
    }

    if(y>4 && x>4){
    	value = buffer[yi][xi];

		if(value == weak){
			flag = false;
			for(int8_t k =-1; k < 2; k++){
				for(int8_t l=-1; l< 2; l++){
					cy = yi + k;
					cx = xi + l;
					if(buffer_condition3(cy,cx,y)){
						if((uint8_t)(buffer[cy][cx]) == strong){
							flag = true;
							break;
						}
					}
				}
				if(flag)
					break;
			}
			if(flag){
				set_pixel(p, strong);
				buffer[yi][xi] = strong;
			}else{
				set_pixel(p, 0);
			}
		}
    }

    if (p.last){
            x = 0;
            y++;
    }
    else{
            x++;
    }
    dst << p;
}

void correction(pixel_stream &src, pixel_stream &dst){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=&src
#pragma HLS INTERFACE axis port=&dst
//#pragma HLS PIPELINE
#pragma HLS inline region recursive

    static uint16_t x = 0;
    static uint16_t y = 0;

    pixel_data p;
    src >> p;
    if (p.user)
            x = y = 0;

    if(y<=4 || x<=4)
    	set_pixel(p, 0);

    if (p.last){
            x = 0;
            y++;
    }
    else{
            x++;
    }
    dst << p;
}
