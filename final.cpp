#include <stdint.h>
#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_video.h>
#include <math.h>

#define WIDTH 1280
#define HEIGHT 720

typedef ap_axiu<32,1,1,1> pixel_data;
typedef hls::stream<pixel_data> pixel_stream;
typedef hls::LineBuffer<5,WIDTH,pixel_data> linebuffer;

uint8_t low_threshold = 2;
uint8_t high_threshold = 23;

uint8_t weak = 30;
uint8_t strong = 255;



bool buffer_condition(int cy, int cx, int y){
        return (cy!=1 || y>=3) && (cy!=0 || y>=4) && cx>=0 && cx<WIDTH && cy>=0 && cy<5;
}

void set_pixel(pixel_data& p, uint32_t intensity){
        p.data = p.data & 0xFF000000 |(intensity << 16) | (intensity << 8) | intensity ;
}

uint32_t get_value(pixel_data& p){
        return p.data & 0x000000FF;
}

void greyscale(pixel_stream &src, pixel_stream &dst){

        static uint16_t x = 0;
        static uint16_t y = 0;

        pixel_data p;
        src >> p;
        if (p.user)
                x = y = 0;

//      if(y>1 && x>1){
                uint32_t a = p.data & 0xFF000000;
                uint32_t b = ((p.data & 0x00FF0000) >> 16);
                uint32_t g = ((p.data & 0x0000FF00) >> 8);
                uint32_t r = (p.data & 0x000000FF);
                uint32_t grayr = (r>>2)+(r>>5);
                uint32_t grayg = (g>>1)+(g>>4);
                uint32_t grayb = (b>>4)+(b>>5);
                uint32_t gray = grayr + grayb + grayg;
                set_pixel(p, gray);
//      }
//  else {
//              make it black
//      }

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

        static uint16_t x = 0;
        static uint16_t y = 0;

        pixel_data p;
        src >> p;
        if (p.user)
                x = y = 0;

        static linebuffer buffer;

        buffer.shift_pixels_up(x);
        buffer.insert_bottom_row(p, x);

        int gauss_kernel[25]={1,4,7,4,1,4,16,26,16,4,7,26,41,26,7,4,16,26,16,4,1,4,7,4,1};
        float value;
        float result = 0.0;

        if(y>1 && x>1){
                for(int i=-2; i<3; i++){
                        for(int j=-2; j<3; j++){
                                int cx = x - 2 + i;
                                int cy = 2 + j;
                                if(buffer_condition(cy, cx, y)){
                                        value = get_value(buffer.getval(cy, cx));
                                        result += value * gauss_kernel[5*(j+2)+i+2];
                                }
                        }
                }
                result/=273;

                set_pixel(p, (uint32_t) result);
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

float convolute(int k[3][3], linebuffer& buffer, int x, int y){

        pixel_data tmp;
        float value;

        float sum = 0.0;
        for(int i=-1; i<2; i++){
                for(int j=-1; j<2; j++){
                        int cx = x - 2 + i;
                        int cy = 2 + j;
                        if(buffer_condition(cy, cx, y)){
                                value = (float)(get_value(buffer.getval(cy, cx)));
                                sum += value * k[i+1][j+1];
                        }
                }
        }
        return sum/2;
}

float sobel_filter(pixel_stream &src, pixel_stream &dst){

        static uint16_t x = 0;
        static uint16_t y = 0;

        pixel_data p;
        src >> p;
        if (p.user)
                x = y = 0;

        static linebuffer buffer;

        buffer.shift_pixels_up(x);
        buffer.insert_bottom_row(p, x);

        int kx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
        int ky[3][3] = {{1,2,1},{0,0,0},{-1,-2,-1}};
        float angle = 0.0;

        if(y>1 && x>1){
                float ix = convolute(kx, buffer, x, y);
                float iy = convolute(ky, buffer, x, y);
                float sqr1 = ix * ix;
                float sqr2 = iy * iy;
                float sum = sqr1 + sqr2;
                uint32_t intensity = (uint32_t) hls::sqrt(sum);
                set_pixel(p, intensity);

                float div = iy / ix;
                angle = hls::atan(div);
        }

        if (p.last){
                x = 0;
                y++;
        }
        else{
                x++;
        }

        dst << p;

        return angle;
}

void suppression(float angle, pixel_stream &src, pixel_stream &dst){

        static uint16_t x = 0;
        static uint16_t y = 0;

        pixel_data p;
        src >> p;
        if (p.user)
                x = y = 0;

        static linebuffer buffer;

        buffer.shift_pixels_up(x);
        buffer.insert_bottom_row(p, x);

        uint32_t result;
        int q = 255, r = 255;
        int cx, cy;
        int xi = x-2, yi = 2;

        if(y>1 && x>1){
                angle = angle * 180 / M_PI;
                if(angle < 0){
                        angle += 180;
                }

                if((0 <= angle < 22.5) or (157.5 <= angle <= 180)){
                        cx = xi + 1;
                        cy = yi;
                        if(buffer_condition(cy,cx, y)){
                                q = get_value(buffer.getval(cy, cx));
                        }
                        cx = xi -1;
                        if(buffer_condition(cy,cx, y)){
                                r = get_value(buffer.getval(cy, cx));
                        }
                }else if(22.5 <= angle < 67.5){
                        cx = xi - 1;
                        cy = yi + 1;
                        if(buffer_condition(cy,cx, y)){
                                q = get_value(buffer.getval(cy, cx));
                        }
                        cx = xi + 1;
                        cy = yi - 1;
                        if(buffer_condition(cy,cx, y)){
                                r = get_value(buffer.getval(cy, cx));
                        }
                }else if(67.5 <= angle < 112.5){
                        cx = xi;
                        cy = yi + 1;
                        if(buffer_condition(cy,cx,y)){
                                q = get_value(buffer.getval(cy, cx));
                        }
                        cy = yi - 1;
                        if(buffer_condition(cy,cx,y)){
                                r = get_value(buffer.getval(cy, cx));
                        }
                }else if(112.5 <= angle < 157.5){
                        cx = xi - 1;
                        cy = yi - 1;
                        if(buffer_condition(cy,cx,y)){
                                q = get_value(buffer.getval(cy, cx));
                        }
                        cx = xi + 1;
                        cy = yi + 1;
                        if(buffer_condition(cy,cx,y)){
                                r = get_value(buffer.getval(cy, cx));
                        }
                }else{
                        std::cout << "error: " << angle << std::endl;
                }

                int value = get_value(p);
                if(value >= q && value >=r ){
                        result = value;
                }else{
                        result = 0;
                }

                set_pixel(p, result);
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

	// we assume the highest pixel value is 255
	// the ratio of high threshold is 0.9
	// the ratio of low threshold is 0.5

    static uint16_t x = 0;
    static uint16_t y = 0;



    pixel_data p;
    src >> p;
    if (p.user)
            x = y = 0;

	if( get_value(p) >= high_threshold){
		set_pixel(p, strong);
	}
	else if(get_value(p) <= high_threshold && get_value(p) >= low_threshold){
		set_pixel(p, weak);
	}
	else if (get_value(p) <= low_threshold){
		set_pixel(p, 0);
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

	// we assume the highest pixel value is 255
	// the ratio of high threshold is 0.9
	// the ratio of low threshold is 0.5
    static uint16_t x = 0;
    static uint16_t y = 0;
    bool flag= false;

    pixel_data p;
    src >> p;
    if (p.user)
            x = y = 0;

    static linebuffer buffer;

    uint8_t value;
    buffer.shift_pixels_up(x);
    buffer.insert_bottom_row(p, x);

    for(int i=-1; i<2; i++){
            for(int j=-1; j<2; j++){
                    int cx = x - 2 + i;
                    int cy = 2 + j;
                    if(buffer_condition(cy, cx, y)){
                    	value = (uint8_t)(get_value(buffer.getval(cy, cx)));
                    	if(value == weak){
                    		flag=f
                        	for(int k =-1; k  < 2; k++){
                        		for(int l=-1; l< 2; l++){
                        			if((uint8_t)(get_value(buffer.getval(cy+ k, cx+l))) == strong){
                        				flag = true;
                        			}
                        		}
                        	}
                        	if(flag == true){
                        		set_pixel(p, strong);
                        	}
                        	else{
                        		set_pixel(p,0);
                        	}

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


